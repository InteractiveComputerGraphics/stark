#pragma once
#include <cstdint>
#include <cassert>

namespace symx
{
    /* Non-owning, bounds-checked view over a contiguous array. */
    template<typename T>
    class View
    {
    public:
        View(T* data, int32_t size) : _data(data), _size(size) {};
        int32_t size() const { return _size; }
        T* data() { return _data; }
        const T* data() const { return _data; }
        T& operator[](int32_t i) { 
            assert(i >= 0 && i < _size && "symx::View index out of bounds"); 
            return _data[i]; 
        }
        const T& operator[](int32_t i) const { 
            assert(i >= 0 && i < _size && "symx::View index out of bounds"); 
            return _data[i]; 
        }
        View slice(int32_t begin, int32_t end) const { 
            assert(begin < end && "symx::View slice begin must be less than end");
            assert(begin >= 0 && end <= _size && "symx::View slice out of bounds"); 
            return View(_data + begin, end - begin); 
        }
        View slice(int32_t begin) const { 
            assert(begin >= 0 && begin < _size && "symx::View slice out of bounds"); 
            return View(_data + begin, _size - begin); 
        }

    private:
        T* _data;
        int32_t _size;
    };
}