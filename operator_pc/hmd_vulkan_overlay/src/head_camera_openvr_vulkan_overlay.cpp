#include <openvr.h>
#include <vulkan/vulkan.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace
{

struct Args
{
  std::string topic = "/rtgr/local/zed/left/image_raw/compressed";
  float width_m = 1.4f;
  float distance_m = 1.2f;
  float x_m = 0.0f;
  float y_m = 0.0f;
  float fps = 15.0f;
};

Args parseArgs(int argc, char ** argv)
{
  Args args;
  for(int i = 1; i < argc; ++i)
  {
    const std::string key = argv[i];
    auto value = [&](const char * name)
    {
      if(i + 1 >= argc) { throw std::runtime_error(std::string("Missing value for ") + name); }
      return std::string(argv[++i]);
    };
    if(key == "--topic") { args.topic = value("--topic"); }
    else if(key == "--width-m") { args.width_m = std::stof(value("--width-m")); }
    else if(key == "--distance-m") { args.distance_m = std::stof(value("--distance-m")); }
    else if(key == "--x-m") { args.x_m = std::stof(value("--x-m")); }
    else if(key == "--y-m") { args.y_m = std::stof(value("--y-m")); }
    else if(key == "--fps") { args.fps = std::stof(value("--fps")); }
    else if(key == "--help" || key == "-h")
    {
      std::cout << "Usage: head_camera_openvr_vulkan_overlay [--topic TOPIC] [--fps FPS]"
                   " [--width-m M] [--distance-m M] [--x-m M] [--y-m M]\n";
      std::exit(0);
    }
  }
  return args;
}

void requireVk(VkResult result, const char * what)
{
  if(result != VK_SUCCESS)
  {
    throw std::runtime_error(std::string(what) + " failed with VkResult " + std::to_string(result));
  }
}

void requireOverlay(vr::EVROverlayError result, vr::IVROverlay * overlay, const char * what)
{
  if(result != vr::VROverlayError_None)
  {
    throw std::runtime_error(std::string(what) + ": " + overlay->GetOverlayErrorNameFromEnum(result));
  }
}

std::vector<std::string> splitExtensions(const std::vector<char> & buffer)
{
  std::vector<std::string> result;
  std::istringstream stream(buffer.data());
  for(std::string item; stream >> item;) { result.push_back(item); }
  return result;
}

std::vector<const char *> extensionPointers(const std::vector<std::string> & extensions)
{
  std::vector<const char *> result;
  result.reserve(extensions.size());
  for(const auto & extension : extensions) { result.push_back(extension.c_str()); }
  return result;
}

vr::HmdMatrix34_t hmdRelativeTransform(float distance_m, float x_m, float y_m)
{
  vr::HmdMatrix34_t transform{};
  transform.m[0][0] = 1.0f;
  transform.m[1][1] = 1.0f;
  transform.m[2][2] = 1.0f;
  transform.m[0][3] = x_m;
  transform.m[1][3] = y_m;
  transform.m[2][3] = -std::abs(distance_m);
  return transform;
}

} // namespace

class HeadCameraVulkanOverlay : public rclcpp::Node
{
public:
  explicit HeadCameraVulkanOverlay(const Args & args)
  : Node("head_camera_openvr_vulkan_overlay"), args_(args)
  {
    initOpenVR();
    initVulkan();
    initOverlay();

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        args_.topic, qos, [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { onImage(*msg); });
    heartbeat_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { heartbeat(); });
    last_frame_time_ = std::chrono::steady_clock::now();
    min_period_ = args_.fps > 0.0f ? std::chrono::duration<double>(1.0 / args_.fps) : std::chrono::duration<double>(0);
    RCLCPP_INFO(get_logger(), "Showing ROS image topic '%s' as Vulkan SteamVR overlay", args_.topic.c_str());
  }

  ~HeadCameraVulkanOverlay() override
  {
    if(overlay_ && overlay_handle_ != vr::k_ulOverlayHandleInvalid)
    {
      overlay_->HideOverlay(overlay_handle_);
      overlay_->ClearOverlayTexture(overlay_handle_);
      overlay_->DestroyOverlay(overlay_handle_);
    }
    if(vr_initialized_)
    {
      vr::VR_Shutdown();
      vr_initialized_ = false;
    }
    destroyVulkan();
  }

private:
  void initOpenVR()
  {
    vr::EVRInitError error = vr::VRInitError_None;
    vr_system_ = vr::VR_Init(&error, vr::VRApplication_Overlay);
    if(error != vr::VRInitError_None)
    {
      throw std::runtime_error(std::string("VR_Init failed: ") + vr::VR_GetVRInitErrorAsEnglishDescription(error));
    }
    vr_initialized_ = true;
    overlay_ = vr::VROverlay();
    compositor_ = vr::VRCompositor();
    if(!vr_system_ || !overlay_ || !compositor_) { throw std::runtime_error("OpenVR interfaces are unavailable"); }
  }

  std::vector<std::string> instanceExtensions()
  {
    const uint32_t size = compositor_->GetVulkanInstanceExtensionsRequired(nullptr, 0);
    if(size == 0) { throw std::runtime_error("SteamVR returned no required Vulkan instance extensions"); }
    std::vector<char> buffer(size, '\0');
    compositor_->GetVulkanInstanceExtensionsRequired(buffer.data(), size);
    return splitExtensions(buffer);
  }

  std::vector<std::string> deviceExtensions()
  {
    const uint32_t size = compositor_->GetVulkanDeviceExtensionsRequired(
        reinterpret_cast<VkPhysicalDevice_T *>(physical_device_), nullptr, 0);
    if(size == 0) { throw std::runtime_error("SteamVR returned no required Vulkan device extensions"); }
    std::vector<char> buffer(size, '\0');
    compositor_->GetVulkanDeviceExtensionsRequired(
        reinterpret_cast<VkPhysicalDevice_T *>(physical_device_), buffer.data(), size);
    return splitExtensions(buffer);
  }

  void initVulkan()
  {
    const auto instance_extensions = instanceExtensions();
    const auto instance_extension_ptrs = extensionPointers(instance_extensions);
    VkApplicationInfo app_info{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app_info.pApplicationName = "RHPS1 ZED HMD Overlay";
    app_info.apiVersion = VK_API_VERSION_1_0;
    VkInstanceCreateInfo instance_info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    instance_info.pApplicationInfo = &app_info;
    instance_info.enabledExtensionCount = static_cast<uint32_t>(instance_extension_ptrs.size());
    instance_info.ppEnabledExtensionNames = instance_extension_ptrs.data();
    requireVk(vkCreateInstance(&instance_info, nullptr, &instance_), "vkCreateInstance");

    uint32_t physical_count = 0;
    requireVk(vkEnumeratePhysicalDevices(instance_, &physical_count, nullptr), "vkEnumeratePhysicalDevices(count)");
    std::vector<VkPhysicalDevice> physical_devices(physical_count);
    requireVk(vkEnumeratePhysicalDevices(instance_, &physical_count, physical_devices.data()), "vkEnumeratePhysicalDevices");
    uint64_t openvr_device = 0;
    vr_system_->GetOutputDevice(&openvr_device, vr::TextureType_Vulkan, reinterpret_cast<VkInstance_T *>(instance_));
    physical_device_ = reinterpret_cast<VkPhysicalDevice>(openvr_device);
    if(std::find(physical_devices.begin(), physical_devices.end(), physical_device_) == physical_devices.end())
    {
      throw std::runtime_error("SteamVR Vulkan output device was not found in this VkInstance");
    }

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physical_device_, &properties);
    vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties_);

    uint32_t queue_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queue_count, nullptr);
    std::vector<VkQueueFamilyProperties> queues(queue_count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queue_count, queues.data());
    for(queue_family_ = 0; queue_family_ < queue_count; ++queue_family_)
    {
      if(queues[queue_family_].queueFlags & VK_QUEUE_GRAPHICS_BIT) { break; }
    }
    if(queue_family_ == queue_count) { throw std::runtime_error("No Vulkan graphics queue found"); }

    const auto device_extensions = deviceExtensions();
    const auto device_extension_ptrs = extensionPointers(device_extensions);
    const float priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queue_info.queueFamilyIndex = queue_family_;
    queue_info.queueCount = 1;
    queue_info.pQueuePriorities = &priority;
    VkDeviceCreateInfo device_info{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    device_info.queueCreateInfoCount = 1;
    device_info.pQueueCreateInfos = &queue_info;
    device_info.enabledExtensionCount = static_cast<uint32_t>(device_extension_ptrs.size());
    device_info.ppEnabledExtensionNames = device_extension_ptrs.data();
    requireVk(vkCreateDevice(physical_device_, &device_info, nullptr, &device_), "vkCreateDevice");
    vkGetDeviceQueue(device_, queue_family_, 0, &queue_);

    VkCommandPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pool_info.queueFamilyIndex = queue_family_;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    requireVk(vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool_), "vkCreateCommandPool");
    VkCommandBufferAllocateInfo command_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    command_info.commandPool = command_pool_;
    command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_info.commandBufferCount = 1;
    requireVk(vkAllocateCommandBuffers(device_, &command_info, &command_buffer_), "vkAllocateCommandBuffers");

    RCLCPP_INFO(get_logger(), "Vulkan texture backend: GPU='%s', queue_family=%u", properties.deviceName, queue_family_);
  }

  void initOverlay()
  {
    const std::string key = "rtgr.rhps1_zed_vulkan_overlay." + std::to_string(::getpid());
    requireOverlay(overlay_->CreateOverlay(key.c_str(), "RHPS1 ZED Vulkan Camera", &overlay_handle_), overlay_, "CreateOverlay");
    requireOverlay(overlay_->SetOverlayWidthInMeters(overlay_handle_, args_.width_m), overlay_, "SetOverlayWidthInMeters");
    requireOverlay(overlay_->SetOverlayAlpha(overlay_handle_, 1.0f), overlay_, "SetOverlayAlpha");
    requireOverlay(overlay_->SetOverlaySortOrder(overlay_handle_, 100), overlay_, "SetOverlaySortOrder");
    requireOverlay(overlay_->SetOverlayFlag(overlay_handle_, vr::VROverlayFlags_IgnoreTextureAlpha, true), overlay_, "SetOverlayFlag");
    auto transform = hmdRelativeTransform(args_.distance_m, args_.x_m, args_.y_m);
    requireOverlay(overlay_->SetOverlayTransformTrackedDeviceRelative(
                       overlay_handle_, vr::k_unTrackedDeviceIndex_Hmd, &transform),
                   overlay_, "SetOverlayTransformTrackedDeviceRelative");
    requireOverlay(overlay_->ShowOverlay(overlay_handle_), overlay_, "ShowOverlay");
  }

  uint32_t memoryType(uint32_t bits, VkMemoryPropertyFlags flags)
  {
    for(uint32_t i = 0; i < memory_properties_.memoryTypeCount; ++i)
    {
      if((bits & (1u << i)) && (memory_properties_.memoryTypes[i].propertyFlags & flags) == flags) { return i; }
    }
    throw std::runtime_error("No compatible Vulkan memory type found");
  }

  void destroyImageResources()
  {
    if(device_ == VK_NULL_HANDLE) { return; }
    vkDeviceWaitIdle(device_);
    if(staging_mapped_) { vkUnmapMemory(device_, staging_memory_); staging_mapped_ = nullptr; }
    if(staging_buffer_) { vkDestroyBuffer(device_, staging_buffer_, nullptr); staging_buffer_ = VK_NULL_HANDLE; }
    if(staging_memory_) { vkFreeMemory(device_, staging_memory_, nullptr); staging_memory_ = VK_NULL_HANDLE; }
    if(image_) { vkDestroyImage(device_, image_, nullptr); image_ = VK_NULL_HANDLE; }
    if(image_memory_) { vkFreeMemory(device_, image_memory_, nullptr); image_memory_ = VK_NULL_HANDLE; }
    width_ = height_ = 0;
    image_initialized_ = false;
  }

  void createImageResources(uint32_t width, uint32_t height)
  {
    destroyImageResources();
    width_ = width;
    height_ = height;
    staging_size_ = static_cast<VkDeviceSize>(width) * height * 4;

    VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    buffer_info.size = staging_size_;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    requireVk(vkCreateBuffer(device_, &buffer_info, nullptr, &staging_buffer_), "vkCreateBuffer");
    VkMemoryRequirements buffer_requirements{};
    vkGetBufferMemoryRequirements(device_, staging_buffer_, &buffer_requirements);
    VkMemoryAllocateInfo buffer_memory_info{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    buffer_memory_info.allocationSize = buffer_requirements.size;
    buffer_memory_info.memoryTypeIndex = memoryType(
        buffer_requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    requireVk(vkAllocateMemory(device_, &buffer_memory_info, nullptr, &staging_memory_), "vkAllocateMemory(staging)");
    requireVk(vkBindBufferMemory(device_, staging_buffer_, staging_memory_, 0), "vkBindBufferMemory");
    requireVk(vkMapMemory(device_, staging_memory_, 0, staging_size_, 0, &staging_mapped_), "vkMapMemory");

    VkImageCreateInfo image_info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
    image_info.extent = {width, height, 1};
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    requireVk(vkCreateImage(device_, &image_info, nullptr, &image_), "vkCreateImage");
    VkMemoryRequirements image_requirements{};
    vkGetImageMemoryRequirements(device_, image_, &image_requirements);
    VkMemoryAllocateInfo image_memory_info{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    image_memory_info.allocationSize = image_requirements.size;
    image_memory_info.memoryTypeIndex = memoryType(image_requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    requireVk(vkAllocateMemory(device_, &image_memory_info, nullptr, &image_memory_), "vkAllocateMemory(image)");
    requireVk(vkBindImageMemory(device_, image_, image_memory_, 0), "vkBindImageMemory");
    RCLCPP_INFO(get_logger(), "Allocated persistent Vulkan overlay texture %ux%u", width_, height_);
  }

  void upload(const cv::Mat & rgba)
  {
    if(rgba.cols <= 0 || rgba.rows <= 0) { return; }
    if(static_cast<uint32_t>(rgba.cols) != width_ || static_cast<uint32_t>(rgba.rows) != height_)
    {
      createImageResources(static_cast<uint32_t>(rgba.cols), static_cast<uint32_t>(rgba.rows));
    }
    const cv::Mat contiguous = rgba.isContinuous() ? rgba : rgba.clone();
    std::memcpy(staging_mapped_, contiguous.data, static_cast<size_t>(staging_size_));

    requireVk(vkResetCommandBuffer(command_buffer_, 0), "vkResetCommandBuffer");
    VkCommandBufferBeginInfo begin_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    requireVk(vkBeginCommandBuffer(command_buffer_, &begin_info), "vkBeginCommandBuffer");

    VkImageMemoryBarrier to_destination{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    to_destination.srcAccessMask = image_initialized_ ? VK_ACCESS_TRANSFER_READ_BIT : 0;
    to_destination.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    to_destination.oldLayout = image_initialized_ ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
    to_destination.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    to_destination.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_destination.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_destination.image = image_;
    to_destination.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    to_destination.subresourceRange.levelCount = 1;
    to_destination.subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(command_buffer_, image_initialized_ ? VK_PIPELINE_STAGE_TRANSFER_BIT : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &to_destination);

    VkBufferImageCopy region{};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageExtent = {width_, height_, 1};
    vkCmdCopyBufferToImage(command_buffer_, staging_buffer_, image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    VkImageMemoryBarrier to_source{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    to_source.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    to_source.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    to_source.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    to_source.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    to_source.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_source.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_source.image = image_;
    to_source.subresourceRange = to_destination.subresourceRange;
    vkCmdPipelineBarrier(command_buffer_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         0, 0, nullptr, 0, nullptr, 1, &to_source);
    requireVk(vkEndCommandBuffer(command_buffer_), "vkEndCommandBuffer");

    VkSubmitInfo submit_info{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer_;
    requireVk(vkQueueSubmit(queue_, 1, &submit_info, VK_NULL_HANDLE), "vkQueueSubmit");
    requireVk(vkQueueWaitIdle(queue_), "vkQueueWaitIdle(upload)");
    image_initialized_ = true;

    vr::VRVulkanTextureData_t data{};
    data.m_nImage = reinterpret_cast<uint64_t>(image_);
    data.m_pDevice = reinterpret_cast<VkDevice_T *>(device_);
    data.m_pPhysicalDevice = reinterpret_cast<VkPhysicalDevice_T *>(physical_device_);
    data.m_pInstance = reinterpret_cast<VkInstance_T *>(instance_);
    data.m_pQueue = reinterpret_cast<VkQueue_T *>(queue_);
    data.m_nQueueFamilyIndex = queue_family_;
    data.m_nWidth = width_;
    data.m_nHeight = height_;
    data.m_nFormat = VK_FORMAT_R8G8B8A8_UNORM;
    data.m_nSampleCount = VK_SAMPLE_COUNT_1_BIT;
    vr::Texture_t texture{&data, vr::TextureType_Vulkan, vr::ColorSpace_Auto};
    requireOverlay(overlay_->SetOverlayTexture(overlay_handle_, &texture), overlay_, "SetOverlayTexture(Vulkan)");
  }

  void onImage(const sensor_msgs::msg::CompressedImage & msg)
  {
    ++frames_received_;
    last_frame_time_ = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if(frames_sent_ > 0 && now - last_submit_time_ < min_period_) { return; }
    try
    {
      const cv::Mat encoded(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t *>(msg.data.data()));
      const cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
      if(bgr.empty()) { throw std::runtime_error("cv::imdecode returned an empty image"); }
      cv::Mat rgba;
      cv::cvtColor(bgr, rgba, cv::COLOR_BGR2RGBA);
      upload(rgba);
      last_submit_time_ = now;
      const auto sent = ++frames_sent_;
      if(sent == 1 || sent % 100 == 0)
      {
        RCLCPP_INFO(get_logger(), "Submitted %zu Vulkan texture frame(s) (%dx%d)", sent, rgba.cols, rgba.rows);
      }
    }
    catch(const std::exception & error)
    {
      ++submit_failures_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Vulkan overlay update failed: %s", error.what());
    }
  }

  void heartbeat()
  {
    vr::VREvent_t event{};
    size_t events = 0;
    while(overlay_ && overlay_->PollNextOverlayEvent(overlay_handle_, &event, sizeof(event))) { ++events; }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - last_frame_time_).count();
    RCLCPP_INFO(get_logger(),
                "Overlay heartbeat: received=%zu, submitted=%zu, failed=%zu, last_frame_age=%ld ms, events=%zu, visible=%s",
                frames_received_.load(), frames_sent_.load(), submit_failures_.load(), static_cast<long>(age_ms), events,
                overlay_ && overlay_->IsOverlayVisible(overlay_handle_) ? "true" : "false");
  }

  void destroyVulkan()
  {
    destroyImageResources();
    if(device_)
    {
      if(command_pool_) { vkDestroyCommandPool(device_, command_pool_, nullptr); }
      vkDestroyDevice(device_, nullptr);
    }
    if(instance_) { vkDestroyInstance(instance_, nullptr); }
    device_ = VK_NULL_HANDLE;
    instance_ = VK_NULL_HANDLE;
  }

private:
  Args args_;
  bool vr_initialized_ = false;
  vr::IVRSystem * vr_system_ = nullptr;
  vr::IVROverlay * overlay_ = nullptr;
  vr::IVRCompositor * compositor_ = nullptr;
  vr::VROverlayHandle_t overlay_handle_ = vr::k_ulOverlayHandleInvalid;

  VkInstance instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
  VkPhysicalDeviceMemoryProperties memory_properties_{};
  VkDevice device_ = VK_NULL_HANDLE;
  VkQueue queue_ = VK_NULL_HANDLE;
  uint32_t queue_family_ = 0;
  VkCommandPool command_pool_ = VK_NULL_HANDLE;
  VkCommandBuffer command_buffer_ = VK_NULL_HANDLE;
  VkBuffer staging_buffer_ = VK_NULL_HANDLE;
  VkDeviceMemory staging_memory_ = VK_NULL_HANDLE;
  void * staging_mapped_ = nullptr;
  VkDeviceSize staging_size_ = 0;
  VkImage image_ = VK_NULL_HANDLE;
  VkDeviceMemory image_memory_ = VK_NULL_HANDLE;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  bool image_initialized_ = false;

  std::chrono::duration<double> min_period_{0};
  std::chrono::steady_clock::time_point last_submit_time_{};
  std::chrono::steady_clock::time_point last_frame_time_{};
  std::atomic<size_t> frames_received_{0};
  std::atomic<size_t> frames_sent_{0};
  std::atomic<size_t> submit_failures_{0};
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  try
  {
    const auto args = parseArgs(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeadCameraVulkanOverlay>(args);
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception & error)
  {
    std::cerr << "head_camera_openvr_vulkan_overlay failed: " << error.what() << std::endl;
    if(rclcpp::ok()) { rclcpp::shutdown(); }
    return 1;
  }
}
