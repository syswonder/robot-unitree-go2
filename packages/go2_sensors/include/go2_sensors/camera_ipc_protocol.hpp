#ifndef GO2_SENSORS__CAMERA_IPC_PROTOCOL_HPP_
#define GO2_SENSORS__CAMERA_IPC_PROTOCOL_HPP_

#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <limits>
#include <string>
#include <string_view>

#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace go2_sensors::camera_ipc
{

constexpr std::uint32_t kMagic = 0x32433247U;  // "G2C2" in little-endian bytes.
constexpr std::uint16_t kVersion = 1U;
constexpr std::size_t kHeaderBytes = 48U;
constexpr std::uint32_t kDefaultMaxJpegBytes = 4U * 1024U * 1024U;
constexpr std::uint32_t kAbsoluteMaxJpegBytes = 16U * 1024U * 1024U;

struct FrameHeader
{
  std::uint64_t sequence{0U};
  std::uint64_t capture_realtime_ns{0U};
  std::uint64_t capture_monotonic_ns{0U};
  std::uint32_t payload_bytes{0U};
  std::uint32_t flags{0U};
  std::uint16_t width_hint{0U};
  std::uint16_t height_hint{0U};
};

enum class HeaderError
{
  kOk,
  kBadMagic,
  kBadVersion,
  kBadHeaderSize,
  kEmptyPayload,
  kPayloadTooLarge,
  kMissingRealtimeStamp,
  kMissingMonotonicStamp,
  kReservedBitsSet,
};

inline const char * header_error_string(const HeaderError error) noexcept
{
  switch (error) {
    case HeaderError::kOk:
      return "ok";
    case HeaderError::kBadMagic:
      return "bad magic";
    case HeaderError::kBadVersion:
      return "unsupported version";
    case HeaderError::kBadHeaderSize:
      return "bad header size";
    case HeaderError::kEmptyPayload:
      return "empty payload";
    case HeaderError::kPayloadTooLarge:
      return "payload too large";
    case HeaderError::kMissingRealtimeStamp:
      return "missing realtime stamp";
    case HeaderError::kMissingMonotonicStamp:
      return "missing monotonic stamp";
    case HeaderError::kReservedBitsSet:
      return "reserved bits are set";
  }
  return "unknown";
}

inline void put_u16(std::uint8_t * const destination, const std::uint16_t value) noexcept
{
  destination[0] = static_cast<std::uint8_t>(value & 0xffU);
  destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

inline void put_u32(std::uint8_t * const destination, const std::uint32_t value) noexcept
{
  for (std::size_t index = 0U; index < 4U; ++index) {
    destination[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

inline void put_u64(std::uint8_t * const destination, const std::uint64_t value) noexcept
{
  for (std::size_t index = 0U; index < 8U; ++index) {
    destination[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

inline std::uint16_t get_u16(const std::uint8_t * const source) noexcept
{
  return static_cast<std::uint16_t>(source[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(source[1]) << 8U);
}

inline std::uint32_t get_u32(const std::uint8_t * const source) noexcept
{
  std::uint32_t value = 0U;
  for (std::size_t index = 0U; index < 4U; ++index) {
    value |= static_cast<std::uint32_t>(source[index]) << (8U * index);
  }
  return value;
}

inline std::uint64_t get_u64(const std::uint8_t * const source) noexcept
{
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value |= static_cast<std::uint64_t>(source[index]) << (8U * index);
  }
  return value;
}

inline std::array<std::uint8_t, kHeaderBytes> encode_header(const FrameHeader & header) noexcept
{
  std::array<std::uint8_t, kHeaderBytes> bytes{};
  put_u32(bytes.data() + 0U, kMagic);
  put_u16(bytes.data() + 4U, kVersion);
  put_u16(bytes.data() + 6U, static_cast<std::uint16_t>(kHeaderBytes));
  put_u64(bytes.data() + 8U, header.sequence);
  put_u64(bytes.data() + 16U, header.capture_realtime_ns);
  put_u64(bytes.data() + 24U, header.capture_monotonic_ns);
  put_u32(bytes.data() + 32U, header.payload_bytes);
  put_u32(bytes.data() + 36U, header.flags);
  put_u16(bytes.data() + 40U, header.width_hint);
  put_u16(bytes.data() + 42U, header.height_hint);
  put_u32(bytes.data() + 44U, 0U);
  return bytes;
}

inline HeaderError decode_header(
  const std::array<std::uint8_t, kHeaderBytes> & bytes,
  const std::uint32_t configured_max_payload,
  FrameHeader & output) noexcept
{
  if (get_u32(bytes.data() + 0U) != kMagic) {
    return HeaderError::kBadMagic;
  }
  if (get_u16(bytes.data() + 4U) != kVersion) {
    return HeaderError::kBadVersion;
  }
  if (get_u16(bytes.data() + 6U) != kHeaderBytes) {
    return HeaderError::kBadHeaderSize;
  }
  output.sequence = get_u64(bytes.data() + 8U);
  output.capture_realtime_ns = get_u64(bytes.data() + 16U);
  output.capture_monotonic_ns = get_u64(bytes.data() + 24U);
  output.payload_bytes = get_u32(bytes.data() + 32U);
  output.flags = get_u32(bytes.data() + 36U);
  output.width_hint = get_u16(bytes.data() + 40U);
  output.height_hint = get_u16(bytes.data() + 42U);
  if (output.payload_bytes == 0U) {
    return HeaderError::kEmptyPayload;
  }
  const auto effective_max =
    configured_max_payload == 0U ? kDefaultMaxJpegBytes : configured_max_payload;
  if (effective_max > kAbsoluteMaxJpegBytes || output.payload_bytes > effective_max) {
    return HeaderError::kPayloadTooLarge;
  }
  if (output.capture_realtime_ns == 0U) {
    return HeaderError::kMissingRealtimeStamp;
  }
  if (output.capture_monotonic_ns == 0U) {
    return HeaderError::kMissingMonotonicStamp;
  }
  if (output.flags != 0U || get_u32(bytes.data() + 44U) != 0U) {
    return HeaderError::kReservedBitsSet;
  }
  return HeaderError::kOk;
}

inline bool looks_like_jpeg(const std::uint8_t * const data, const std::size_t size) noexcept
{
  return data != nullptr && size >= 4U && data[0] == 0xffU && data[1] == 0xd8U &&
         data[size - 2U] == 0xffU && data[size - 1U] == 0xd9U;
}

inline bool is_start_of_frame_marker(const std::uint8_t marker) noexcept
{
  return (marker >= 0xc0U && marker <= 0xc3U) ||
         (marker >= 0xc5U && marker <= 0xc7U) ||
         (marker >= 0xc9U && marker <= 0xcbU) ||
         (marker >= 0xcdU && marker <= 0xcfU);
}

inline bool jpeg_dimensions(
  const std::uint8_t * const data, const std::size_t size,
  std::uint16_t & width, std::uint16_t & height) noexcept
{
  width = 0U;
  height = 0U;
  if (!looks_like_jpeg(data, size)) {
    return false;
  }
  std::size_t offset = 2U;
  while (offset + 1U < size) {
    if (data[offset] != 0xffU) {
      return false;
    }
    while (offset < size && data[offset] == 0xffU) {
      ++offset;
    }
    if (offset >= size) {
      return false;
    }
    const std::uint8_t marker = data[offset++];
    if (marker == 0xd9U || marker == 0xdaU) {
      return false;
    }
    if (marker == 0x01U || (marker >= 0xd0U && marker <= 0xd7U)) {
      continue;
    }
    if (offset + 2U > size) {
      return false;
    }
    const std::uint16_t segment_size = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[offset]) << 8U) |
      static_cast<std::uint16_t>(data[offset + 1U]));
    if (segment_size < 2U || static_cast<std::size_t>(segment_size) > size - offset) {
      return false;
    }
    if (is_start_of_frame_marker(marker)) {
      if (segment_size < 8U) {
        return false;
      }
      height = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[offset + 3U]) << 8U) |
        static_cast<std::uint16_t>(data[offset + 4U]));
      width = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[offset + 5U]) << 8U) |
        static_cast<std::uint16_t>(data[offset + 6U]));
      return width > 0U && height > 0U;
    }
    offset += static_cast<std::size_t>(segment_size);
  }
  return false;
}

inline std::uint64_t clock_nanoseconds(const clockid_t clock_id) noexcept
{
  timespec time_value{};
  if (::clock_gettime(clock_id, &time_value) != 0 || time_value.tv_sec < 0) {
    return 0U;
  }
  const auto seconds = static_cast<std::uint64_t>(time_value.tv_sec);
  const auto nanoseconds = static_cast<std::uint64_t>(time_value.tv_nsec);
  constexpr auto kNanosecondsPerSecond = 1000000000ULL;
  if (seconds > (std::numeric_limits<std::uint64_t>::max() - nanoseconds) /
    kNanosecondsPerSecond)
  {
    return 0U;
  }
  return seconds * kNanosecondsPerSecond + nanoseconds;
}

inline bool wait_for_fd(
  const int file_descriptor, const short events, const int timeout_ms,
  std::string & error) noexcept
{
  pollfd descriptor{file_descriptor, events, 0};
  while (true) {
    const int result = ::poll(&descriptor, 1, timeout_ms);
    if (result > 0) {
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        error = "socket closed";
        return false;
      }
      if ((descriptor.revents & events) != 0) {
        return true;
      }
      error = "unexpected socket poll event";
      return false;
    }
    if (result == 0) {
      error = "socket timeout";
      return false;
    }
    if (errno != EINTR) {
      error = std::string("poll failed: ") + std::strerror(errno);
      return false;
    }
  }
}

inline bool read_exact(
  const int file_descriptor, void * const destination, const std::size_t size,
  const int timeout_ms, std::string & error) noexcept
{
  auto * cursor = static_cast<std::uint8_t *>(destination);
  std::size_t remaining = size;
  while (remaining > 0U) {
    if (!wait_for_fd(file_descriptor, POLLIN, timeout_ms, error)) {
      return false;
    }
    const ssize_t count = ::read(file_descriptor, cursor, remaining);
    if (count > 0) {
      const auto consumed = static_cast<std::size_t>(count);
      cursor += consumed;
      remaining -= consumed;
      continue;
    }
    if (count == 0) {
      error = "peer disconnected";
      return false;
    }
    if (errno != EINTR) {
      error = std::string("read failed: ") + std::strerror(errno);
      return false;
    }
  }
  return true;
}

inline bool write_exact(
  const int file_descriptor, const void * const source, const std::size_t size,
  const int timeout_ms, std::string & error) noexcept
{
  const auto * cursor = static_cast<const std::uint8_t *>(source);
  std::size_t remaining = size;
  while (remaining > 0U) {
    if (!wait_for_fd(file_descriptor, POLLOUT, timeout_ms, error)) {
      return false;
    }
    // The sole production writer (the SDK daemon) ignores SIGPIPE before this
    // helper is used, so write(2) remains safe when the ROS peer disconnects.
    const ssize_t count = ::write(file_descriptor, cursor, remaining);
    if (count > 0) {
      const auto consumed = static_cast<std::size_t>(count);
      cursor += consumed;
      remaining -= consumed;
      continue;
    }
    if (count == 0) {
      error = "peer accepted no data";
      return false;
    }
    if (errno != EINTR) {
      error = std::string("write failed: ") + std::strerror(errno);
      return false;
    }
  }
  return true;
}

inline std::string default_socket_path()
{
  return "/run/user/" + std::to_string(static_cast<unsigned long>(::getuid())) +
         "/robonix-go2/camera.sock";
}

}  // namespace go2_sensors::camera_ipc

#endif  // GO2_SENSORS__CAMERA_IPC_PROTOCOL_HPP_
