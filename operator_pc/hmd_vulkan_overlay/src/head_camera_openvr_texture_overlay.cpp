#include <GL/glew.h>
#include <SDL.h>
#include <openvr.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct Args
{
  std::string topic = "/rtgr/sim/head_camera/rgb/image_raw";
  float width_m = 1.4f;
  float distance_m = 1.2f;
  // overlay 相对头显中心的水平/垂直偏移，用来临时校正画面位置。
  float x_m = 0.0f;
  float y_m = 0.0f;
};

Args parseArgs(int argc, char ** argv)
{
  Args args;
  for(int i = 1; i < argc; ++i)
  {
    const std::string key = argv[i];
    auto needValue = [&](const char * name) -> const char *
    {
      if(i + 1 >= argc)
      {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };
    if(key == "--topic")
    {
      args.topic = needValue("--topic");
    }
    else if(key == "--width-m")
    {
      args.width_m = std::stof(needValue("--width-m"));
    }
    else if(key == "--distance-m")
    {
      args.distance_m = std::stof(needValue("--distance-m"));
    }
    else if(key == "--x-m")
    {
      args.x_m = std::stof(needValue("--x-m"));
    }
    else if(key == "--y-m")
    {
      args.y_m = std::stof(needValue("--y-m"));
    }
    else if(key == "--help" || key == "-h")
    {
      std::cout << "Usage: head_camera_openvr_texture_overlay [--topic TOPIC] [--width-m M] [--distance-m M]"
                   " [--x-m M] [--y-m M]\n";
      std::exit(0);
    }
  }
  return args;
}

void checkOpenVRError(vr::EVROverlayError err, const std::string & what)
{
  if(err != vr::VROverlayError_None)
  {
    throw std::runtime_error(what + ": " + vr::VROverlay()->GetOverlayErrorNameFromEnum(err));
  }
}

vr::HmdMatrix34_t hmdRelativeTransform(float distance_m, float x_m, float y_m)
{
  vr::HmdMatrix34_t transform{};
  // 这是一个跟随头显的 2D texture overlay，不是真正的双目立体渲染。
  transform.m[0][0] = 1.0f;
  transform.m[0][1] = 0.0f;
  transform.m[0][2] = 0.0f;
  transform.m[0][3] = x_m;
  transform.m[1][0] = 0.0f;
  transform.m[1][1] = 1.0f;
  transform.m[1][2] = 0.0f;
  transform.m[1][3] = y_m;
  transform.m[2][0] = 0.0f;
  transform.m[2][1] = 0.0f;
  transform.m[2][2] = 1.0f;
  transform.m[2][3] = -std::abs(distance_m);
  return transform;
}

} // 匿名命名空间

class HeadCameraTextureOverlay : public rclcpp::Node
{
public:
  explicit HeadCameraTextureOverlay(const Args & args)
  : Node("head_camera_openvr_texture_overlay"), args_(args)
  {
    initGL();
    initOpenVR();

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        args_.topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(*msg); });

    last_frame_time_ = std::chrono::steady_clock::now();
    heartbeat_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { heartbeat(); });

    RCLCPP_INFO(get_logger(), "Showing ROS image topic '%s' as OpenGL SteamVR overlay", args_.topic.c_str());
  }

  ~HeadCameraTextureOverlay() override
  {
    if(overlay_ && overlay_handle_ != vr::k_ulOverlayHandleInvalid)
    {
      overlay_->HideOverlay(overlay_handle_);
      overlay_->DestroyOverlay(overlay_handle_);
    }
    if(texture_)
    {
      glDeleteTextures(1, &texture_);
    }
    if(gl_context_)
    {
      SDL_GL_DeleteContext(gl_context_);
    }
    if(window_)
    {
      SDL_DestroyWindow(window_);
    }
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    if(vr_initialized_)
    {
      vr::VR_Shutdown();
    }
  }

private:
  void initGL()
  {
    if(SDL_InitSubSystem(SDL_INIT_VIDEO) != 0)
    {
      throw std::runtime_error(std::string("SDL_InitSubSystem failed: ") + SDL_GetError());
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    window_ = SDL_CreateWindow("head_camera_openvr_texture_overlay", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 64,
                               64, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    if(!window_)
    {
      throw std::runtime_error(std::string("SDL_CreateWindow failed: ") + SDL_GetError());
    }
    gl_context_ = SDL_GL_CreateContext(window_);
    if(!gl_context_)
    {
      throw std::runtime_error(std::string("SDL_GL_CreateContext failed: ") + SDL_GetError());
    }
    SDL_GL_MakeCurrent(window_, gl_context_);
    glewExperimental = GL_TRUE;
    const auto glew_status = glewInit();
    if(glew_status != GLEW_OK)
    {
      throw std::runtime_error(std::string("glewInit failed: ")
                               + reinterpret_cast<const char *>(glewGetErrorString(glew_status)));
    }
    RCLCPP_INFO(get_logger(),
                "OpenGL context: vendor='%s', renderer='%s', version='%s'",
                reinterpret_cast<const char *>(glGetString(GL_VENDOR)),
                reinterpret_cast<const char *>(glGetString(GL_RENDERER)),
                reinterpret_cast<const char *>(glGetString(GL_VERSION)));
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }

  void initOpenVR()
  {
    vr::EVRInitError init_error = vr::VRInitError_None;
    vr::VR_Init(&init_error, vr::VRApplication_Overlay);
    if(init_error != vr::VRInitError_None)
    {
      throw std::runtime_error(std::string("VR_Init failed: ") + vr::VR_GetVRInitErrorAsEnglishDescription(init_error));
    }
    vr_initialized_ = true;

    overlay_ = vr::VROverlay();
    if(!overlay_)
    {
      throw std::runtime_error("VROverlay interface is not available");
    }

    const char * key = "rtgr.head_camera_texture_overlay";
    auto err = overlay_->FindOverlay(key, &overlay_handle_);
    if(err != vr::VROverlayError_None)
    {
      err = overlay_->CreateOverlay(key, "RTGR MuJoCo Head Camera Texture", &overlay_handle_);
      checkOpenVRError(err, "CreateOverlay");
    }
    checkOpenVRError(overlay_->SetOverlayWidthInMeters(overlay_handle_, args_.width_m), "SetOverlayWidthInMeters");
    checkOpenVRError(overlay_->SetOverlayAlpha(overlay_handle_, 1.0f), "SetOverlayAlpha");
    checkOpenVRError(overlay_->SetOverlaySortOrder(overlay_handle_, 100), "SetOverlaySortOrder");
    checkOpenVRError(overlay_->SetOverlayFlag(overlay_handle_, vr::VROverlayFlags_IgnoreTextureAlpha, true),
                     "SetOverlayFlag");
    auto transform = hmdRelativeTransform(args_.distance_m, args_.x_m, args_.y_m);
    checkOpenVRError(overlay_->SetOverlayTransformTrackedDeviceRelative(overlay_handle_, vr::k_unTrackedDeviceIndex_Hmd,
                                                                        &transform),
                     "SetOverlayTransformTrackedDeviceRelative");
    checkOpenVRError(overlay_->ShowOverlay(overlay_handle_), "ShowOverlay");
  }

  void onImage(const sensor_msgs::msg::Image & msg)
  {
    ++frames_received_;
    last_frame_time_ = std::chrono::steady_clock::now();

    if(msg.encoding != "rgb8" && msg.encoding != "rgba8")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Unsupported image encoding '%s'", msg.encoding.c_str());
      return;
    }

    const uint32_t channels = msg.encoding == "rgba8" ? 4u : 3u;
    if(msg.width == 0 || msg.height == 0 || msg.step < msg.width * channels)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Invalid image layout");
      return;
    }

    rgba_.resize(static_cast<size_t>(msg.width) * msg.height * 4);
    for(uint32_t y = 0; y < msg.height; ++y)
    {
      const auto * src = msg.data.data() + static_cast<size_t>(y) * msg.step;
      // OpenGL 纹理坐标和 ROS image 行方向相反，这里上下翻转一次。
      auto * dst = rgba_.data() + static_cast<size_t>(msg.height - 1 - y) * msg.width * 4;
      for(uint32_t x = 0; x < msg.width; ++x)
      {
        dst[4 * x + 0] = src[channels * x + 0];
        dst[4 * x + 1] = src[channels * x + 1];
        dst[4 * x + 2] = src[channels * x + 2];
        dst[4 * x + 3] = channels == 4 ? src[channels * x + 3] : 255;
      }
    }

    SDL_GL_MakeCurrent(window_, gl_context_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    if(msg.width != texture_width_ || msg.height != texture_height_)
    {
      texture_width_ = msg.width;
      texture_height_ = msg.height;
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, static_cast<GLsizei>(texture_width_), static_cast<GLsizei>(texture_height_),
                   0, GL_RGBA, GL_UNSIGNED_BYTE, rgba_.data());
    }
    else
    {
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(texture_width_),
                      static_cast<GLsizei>(texture_height_), GL_RGBA, GL_UNSIGNED_BYTE, rgba_.data());
    }
    glFlush();

    if(!texture_diagnostics_logged_)
    {
      texture_diagnostics_logged_ = true;
      const auto gl_error = glGetError();
      RCLCPP_INFO(get_logger(),
                  "OpenGL texture check: id=%u, valid=%s, size=%ux%u, gl_error=0x%x",
                  texture_,
                  glIsTexture(texture_) == GL_TRUE ? "true" : "false",
                  texture_width_,
                  texture_height_,
                  static_cast<unsigned int>(gl_error));
    }

    vr::Texture_t vr_texture{};
    vr_texture.handle = reinterpret_cast<void *>(static_cast<uintptr_t>(texture_));
    vr_texture.eType = vr::TextureType_OpenGL;
    vr_texture.eColorSpace = vr::ColorSpace_Auto;
    const auto err = overlay_->SetOverlayTexture(overlay_handle_, &vr_texture);
    if(err != vr::VROverlayError_None)
    {
      ++submit_failures_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "SetOverlayTexture failed: %s",
                            overlay_->GetOverlayErrorNameFromEnum(err));
      return;
    }
    overlay_->ShowOverlay(overlay_handle_);

    const auto sent = ++frames_sent_;
    if(sent == 1 || sent % 100 == 0)
    {
      RCLCPP_INFO(get_logger(), "Submitted %zu texture frame(s) to OpenVR overlay (%ux%u)", sent, msg.width, msg.height);
    }
  }

  void heartbeat()
  {
    // 定期处理事件，并区分 ROS 停帧、提交失败和 SteamVR 不再刷新三种情况。
    vr::VREvent_t event{};
    size_t event_count = 0;
    while(overlay_ && overlay_handle_ != vr::k_ulOverlayHandleInvalid
          && overlay_->PollNextOverlayEvent(overlay_handle_, &event, sizeof(event)))
    {
      ++event_count;
    }

    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - last_frame_time_)
                            .count();
    const bool visible = overlay_ && overlay_handle_ != vr::k_ulOverlayHandleInvalid
                         && overlay_->IsOverlayVisible(overlay_handle_);
    RCLCPP_INFO(get_logger(),
                "Overlay heartbeat: received=%zu, submitted=%zu, failed=%zu, last_frame_age=%ld ms, events=%zu, visible=%s",
                frames_received_.load(),
                frames_sent_.load(),
                submit_failures_.load(),
                static_cast<long>(age_ms),
                event_count,
                visible ? "true" : "false");
  }

private:
  Args args_;
  SDL_Window * window_ = nullptr;
  SDL_GLContext gl_context_ = nullptr;
  bool vr_initialized_ = false;
  vr::IVROverlay * overlay_ = nullptr;
  vr::VROverlayHandle_t overlay_handle_ = vr::k_ulOverlayHandleInvalid;
  GLuint texture_ = 0;
  bool texture_diagnostics_logged_ = false;
  uint32_t texture_width_ = 0;
  uint32_t texture_height_ = 0;
  std::vector<uint8_t> rgba_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::chrono::steady_clock::time_point last_frame_time_;
  std::atomic<size_t> frames_received_{0};
  std::atomic<size_t> frames_sent_{0};
  std::atomic<size_t> submit_failures_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  try
  {
    const auto args = parseArgs(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeadCameraTextureOverlay>(args);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception & e)
  {
    std::cerr << "head_camera_openvr_texture_overlay failed: " << e.what() << std::endl;
    return 1;
  }
}
