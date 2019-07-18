//
// Created by jhwangbo on 18. 1. 30.
//

#ifndef RAISIM_HELPER_HPP
#define RAISIM_HELPER_HPP
#ifdef __unix__
#include <unistd.h>
#endif
#ifdef WINDOWS
#include <windows.h>
#endif

#include <vector>
#include <stdexcept>
#include <mm_malloc.h>
#include <stddef.h>
#include "raisim_message.hpp"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

namespace raisim {

inline void MSLEEP(int sleepMs)
{
#ifdef __unix__
  usleep(sleepMs * 1000);   // usleep takes sleep time in us (1 millionth of a second)
#endif
#ifdef WINDOWS
  Sleep(sleepMs);
#endif
}

inline char separator() {
#ifdef WINDOWS
  return '\\';
#else
  return '/';
#endif
}

/* returns the file name without extension */
inline std::string getBaseFileName(const std::string& fullPath) {
  std::string extension = fullPath.substr(fullPath.find_last_of('.') + 1);
  return fullPath.substr(fullPath.find_last_of(separator()) + 1,
                     fullPath.size() - fullPath.find_last_of(separator()) - extension.size() - 2);
}

/* returns only the name of the directory one directory higher than the directory of the file */
inline std::string getTopDirectory(const std::string& fullPath) {
  size_t lastSep = fullPath.find_last_of(separator());
  size_t secondLastSep = fullPath.substr(0, lastSep).find_last_of(separator());
  return fullPath.substr(secondLastSep+1, lastSep-secondLastSep-1);
}

/* returns the file name including the extension */
inline std::string getFileName(const std::string& fullPath) {
  return fullPath.substr(fullPath.find_last_of(separator()) + 1);
}

/* returns only the file extension */
inline std::string getFileExtension(const std::string& fullPath) {
  return fullPath.substr(fullPath.find_last_of('.') + 1);
}

/* returns the full path containing the file (remove the file name in the full path) */
inline std::string getPathName(const std::string &fullPath) {
  size_t i = fullPath.rfind(separator(), fullPath.length());
  if (i != std::string::npos)
    return (fullPath.substr(0, i));

  return ("");
}

/* returns true if the file exists, returns false if not */
inline bool fileExists (const std::string& name) {
  std::ifstream f(name.c_str());
  return f.good();
}

inline bool directoryExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

void read_png_file(const char *file_name, int &width, int &height, std::vector<double> &values, double scale, double zOffset);

template<typename T, std::size_t Alignment>
class AlignedAllocator {
 public:

  typedef T *pointer;
  typedef const T *const_pointer;
  typedef T &reference;
  typedef const T &const_reference;
  typedef T value_type;
  typedef std::size_t size_type;
  typedef ptrdiff_t difference_type;

  T *address(T &r) const {
    return &r;
  }

  const T *address(const T &s) const {
    return &s;
  }

  std::size_t max_size() const {
    return (static_cast<std::size_t>(0) - static_cast<std::size_t>(1)) / sizeof(T);
  }

  template<typename U>
  struct rebind {
    typedef AlignedAllocator<U, Alignment> other;
  };

  bool operator!=(const AlignedAllocator &other) const {
    return !(*this == other);
  }

  void construct(T *const p, const T &t) const {
    void *const pv = static_cast<void *>(p);

    new(pv) T(t);
  }

  void destroy(T *const p) const {
    p->~T();
  }

  bool operator==(const AlignedAllocator &other) const {
    return true;
  }

  AlignedAllocator() = default;

  AlignedAllocator(const AlignedAllocator &) {}

  template<typename U>
  AlignedAllocator(const AlignedAllocator<U, Alignment> &) {}

  ~AlignedAllocator() {}

  T *allocate(const std::size_t n) const {
    if (n == 0)
      return NULL;

    void *const pv = _mm_malloc(n * sizeof(T), Alignment);

    if (pv == NULL) {
      throw std::bad_alloc();
    }

    return static_cast<T *>(pv);
  }

  void deallocate(T *const p, const std::size_t n) const {
    _mm_free(p);
  }

  template<typename U>
  T *allocate(const std::size_t n, const U * /* const hint */) const {
    return allocate(n);
  }

 private:
  AlignedAllocator &operator=(const AlignedAllocator &);
};

}

#endif //RAISIM_HELPER_HPP
