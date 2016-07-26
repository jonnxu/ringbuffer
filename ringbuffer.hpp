//
// dsa is a utility library of data structures and algorithms built with C++11.
// This file (spinlock.hpp) is part of the dsa project.
//
// spinlock; a user-space spinlock implementation for C++11 or later.
//
// author: Dalton Woodard
// contact: daltonmwoodard@gmail.com
// repository: https://github.com/daltonwoodard/spinlock.hpp
// license:
//
// Copyright (c) 2016 DaltonWoodard. See the COPYRIGHT.md file at the top-level
// directory or at the listed source repository for details.
//
//      Licensed under the Apache License. Version 2.0:
//          https://www.apache.org/licenses/LICENSE-2.0
//      or the MIT License:
//          https://opensource.org/licenses/MIT
//      at the licensee's option. This file may not be copied, modified, or
//      distributed except according to those terms.
//

#ifndef DSA_RINGBUFFER_HPP
#define DSA_RINGBUFFER_HPP

// note:
//      This implementation is NOT threadsafe. Please see
//      https://github.com/daltonwoodard/atomic_ringbuffer.git
//      for a thread-safe version implemented with C++ atomic
//      primitives.
//
// note:
//      While this type is backed by a std::array object,
//      and can therefore exist entirely on the stack, it
//      is possible for this type to be heap allocated so
//      as to avoid stack size limitations. The use of the
//      std::array backing object is to provide data locality
//      locality of the buffered data with respect to the
//      managing ringbuffer object.
//
//  note:
//      If and only if the stored object type T has strong exception safe
//      constructors is the following guaranteed:
//
//      If this object is successfull constructed, then throughout the
//      object's lifetime:
//
//          i.  all view operations (front/back) are guaranteed as nothrow.
//          ii. read opeartions provide the basic exception safety guarantee;
//
//     Notice that the read operations modify the data structure (by removing
//     elements from the front); hence they are, in actuallity, writes.
//
//     All of the above are documented as such in source. 

#include <array>        // std::array
#include <type_traits>  // std::remove_cv
#include <utility>      // std::forward, std::move


namespace dsa
{
    template <typename T, std::size_t N>
    class ringbuffer
    {
        static_assert (N > 0, "empty ringbuffer is not allowed");

    private:
        struct memblock
        {
            alignas (alignof (typename std::remove_cv <T>::type))
                unsigned char data [sizeof (typename std::remove_cv <T>::type)];
        };

        using backing_type            = std::array <memblock, N>;
        using backing_pointer         = typename backing_type::pointer;
        using backing_const_pointer   = typename backing_type::const_pointer;
        using backing_reference       = typename backing_type::reference;
        using backing_const_reference = typename backing_type::const_reference;

        using unqualified_type            = typename std::remove_cv <T>::type;
        using unqualified_pointer         = unqualified_type *;
        using unqualified_const_pointer   = unqualified_type const *;
        using unqualified_reference       = unqualified_type *;
        using unqualified_const_reference = unqualified_type const *;

        backing_type _buffer;
        std::size_t _buffered;

        backing_pointer const _first = &_buffer [0];
        backing_pointer const _last  = &_buffer [N - 1];

        template <typename U>
        class iterator_impl;

        iterator_impl <T> _write_location {
            reinterpret_cast <unqualified_pointer> (&_buffer [0]),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last)
        };

        iterator_impl <T> _read_location  {
            reinterpret_cast <unqualified_pointer> (&_buffer [0]),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last)
        };

        template <typename U>
        class iterator_impl : public std::iterator <
            std::random_access_iterator_tag,
            typename std::remove_cv <U>::type,
            std::ptrdiff_t,
            typename std::remove_cv <U>::type *,
            typename std::remove_cv <U>::type &
        >
        {
        private:
            using unqualified_type = typename std::remove_cv <U>::type;
            using iter_type = std::iterator <
                std::random_access_iterator_tag,
                unqualified_type,
                std::ptrdiff_t,
                unqualified_type *,
                unqualified_type &
            >;

            U * _iter;
            unqualified_type * const _first;
            unqualified_type * const _last;

        public:
            using difference_type   = typename iter_type::difference_type;
            using value_type        = typename iter_type::value_type;
            using pointer           = value_type *;
            using const_pointer     = value_type const *;
            using reference         = value_type &;
            using const_reference   = value_type const &;
            using iterator_category = typename iter_type::iterator_category;

            iterator_impl (void) = delete;

            iterator_impl (pointer p, pointer first, pointer last)
                noexcept
                : _iter  {p}
                , _first {first}
                , _last  {last}
            {}

            void swap (iterator_impl & other)
            {
                std::swap (this->_iter, other._iter);
                std::swap (this->_first, other._first);
                std::swap (this->_last, other._last);
            }

            iterator_impl & operator++ (void)
            {
                if (_iter == _last) {
                    _iter = _first;
                } else {
                    _iter += 1;
                }

                return *this;
            }

            iterator_impl & operator-- (void)
            {
                if (_iter == _first) {
                    _iter = _last;
                } else {
                    _iter -= 1;
                }

                return *this;
            }

            iterator_impl operator++ (int)
            {
                auto const tmp {*this};

                if (_iter == _last) {
                    _iter = _first;
                } else {
                    _iter += 1;
                }

                return tmp;
            }

            iterator_impl operator-- (int)
            {
                auto const tmp {*this};

                if (_iter == _first) {
                    _iter = _last;
                } else {
                    _iter -= 1;
                }

                return tmp;
            }

            iterator_impl & operator+= (difference_type n)
            {
                if (n >= N || -n >= N) {
                    n = n % N;
                }

                if (n >= 0) {
                    if (_iter > _last - n) {
                        _iter = _first + (n - 1 - (_last - _iter));
                    } else {
                        _iter = _iter + n;
                    }
                } else {
                    auto const m {-n};
                    if (_iter < _first + m) {
                        _iter = _last - (m - 1 - (_iter - _first));
                    } else {
                        _iter = _iter - m;
                    }
                }

                return *this;
            }

            iterator_impl & operator-= (difference_type n)
            {
                return this->operator+= (-n);
            }

            iterator_impl operator+ (difference_type n) const
            {
                auto tmp = *this;
                return (tmp += n); 
            }

            iterator_impl operator- (difference_type n) const
            {
                auto tmp = *this;
                return (tmp -= n);
            }

            difference_type operator- (iterator_impl const & rhs) const
            {
                return _iter - rhs._iter;
            }

            bool operator== (iterator_impl const & rhs) const
            {
                return _iter == rhs._iter;
            }

            bool operator!= (iterator_impl const & rhs) const
            {
                return _iter != rhs._iter;
            }

            bool operator< (iterator_impl const & rhs) const
            {
                return _iter < rhs._iter;
            }

            bool operator> (iterator_impl const & rhs) const
            {
                return _iter > rhs._iter;
            }

            bool operator<= (iterator_impl const & rhs) const
            {
                return _iter <= rhs._iter;
            }

            bool operator>= (iterator_impl const & rhs) const
            {
                return _iter >= rhs._iter;
            }

            reference operator* (void)
            {
                return *_iter;
            }

            const_reference operator* (void) const
            {
                return *_iter;
            }

            reference operator[] (difference_type n)
            {
                return *(*this + n);
            }

            const_reference operator[] (difference_type n) const
            {
                return *(*this + n);
            }
        };

    public:
        using value_type      = unqualified_type;
        using size_type       = std::size_t;
        using difference_type = std::ptrdiff_t;
        using pointer         = unqualified_pointer;
        using const_pointer   = unqualified_const_pointer;
        using reference       = unqualified_reference;
        using const_reference = unqualified_const_reference;

        using iterator        = iterator_impl <T>;
        using const_iterator  = iterator_impl <T const>;
        using reverse_iterator       = std::reverse_iterator <iterator>;
        using const_reverse_iterator = std::reverse_iterator <const_iterator>;

        ringbuffer (void) noexcept
            : _buffer   {}
            , _buffered {0}
        {}

        ringbuffer (ringbuffer const & other)
            : _buffer   {}
            , _buffered {other._buffered}
        {
            auto ti = this->_write_location;
            auto oi = other.cbegin ();
            while (oi != other.cend ()) {
                *ti++ = *oi++;
            }

            this->_write_location += this->_buffered;
        }

        ~ringbuffer (void) noexcept (noexcept (~T ()))
        {
            for (auto it = _read_location; it != _write_location; ++it) {
                (*it).~T ();
            }
        }

        /* checks whether the buffer is emtpy */
        bool empty (void) const noexcept
        {
            return _buffered != 0;
        }

        /* returns the current number of elements stored in the buffer */
        std::size_t size (void) const noexcept
        {
            return _buffered;
        }

        // obtain the remaining capacity to add objects
        // to buffer.
        //
        // -- nothrow
        //
        std::size_t capacity (void) const noexcept
        {
            return N - _buffered;
        }

        /* returns an iterator the start of the buffer */
        iterator begin (void) noexcept
        {
            return _read_location;
        }

        /* returns an iterator the end of the buffer */
        iterator end (void) noexcept
        {
            return _write_location;
        }

        /* returns a const iterator the start of the buffer */
        const_iterator begin (void) const noexcept
        {
            return this->cbegin ();
        }

        /* returns a const iterator the end of the buffer */
        const_iterator end (void) const noexcept
        {
            return this->cend ();
        }

        /* returns a const iterator the start of the buffer */
        const_iterator cbegin (void) const noexcept
        {
            return const_iterator {&*_read_location, _first, _last};
        }

        /* returns a const iterator the end of the buffer */
        const_iterator cend (void) const noexcept
        {
            return const_iterator {&*_write_location, _first, _last};
        }

        /* returns a reverse iterator to the start of the reversed buffer */
        reverse_iterator rbegin (void) noexcept
        {
            return reverse_iterator {this->end ()};
        }

        /* returns a reverse iterator to the end of the reversed buffer */
        reverse_iterator rend (void) noexcept
        {
            return reverse_iterator {this->begin ()};
        }

        /*
         * returns a const reverse iterator to the start of the reversed buffer
         */
        const_reverse_iterator rbegin (void) const noexcept
        {
            return this->crbegin ();
        }

        /* returns a const reverse iterator to the end of the reversed buffer */
        const_reverse_iterator rend (void) const noexcept
        {
            return this->crend ();
        }

        /*
         * returns a const reverse iterator to the start of the reversed buffer
         */
        const_iterator crbegin (void) const noexcept
        {
            return const_reverse_iterator {this->cend ()};
        }

        /* returns a const reverse iterator to the end of the reversed buffer */
        const_iterator crend (void) const noexcept
        {
            return const_reverse_iterator {this->cbegin ()};
        }

        /* returns a reference to the first element in the buffer */
        reference front (void) noexcept
        {
            return _read_location [0];
        }

        /* returns a reference to the first element in the buffer */
        const_reference front (void) const noexcept
        {
            return _read_location [0];
        }

        /* returns a reference to the last element in the buffer */
        reference back (void) noexcept
        {
            if (_buffered) {
                return _read_location [_buffered - 1];
            } else {
                return _read_location [0];
            }
        }

        /* returns a reference to the last element in the buffer */
        const_reference back (void) const noexcept
        {
            if (_buffered) {
                return _read_location [_buffered - 1];
            } else {
                return _read_location [0];
            }
        }

        /*
         * adds an object to the buffer if room is available; throws an
         * exception of type std::runtime_error if no room is available in the
         * buffer.
         */
        void push (value_type const & v)
        {
            if (_buffered < N) {
                *_write_location = v;
                _write_location += 1;
            } else {
                throw std::runtime_error {"emplace on full buffer"};
            }
        }

        /*
         * adds an object to the buffer if room is available; throws an
         * exception of type std::runtime_error if no room is available in the
         * buffer.
         */
        void push (value_type && v)
        {
            if (_buffered < N) {
                *_write_location = std::move (v);
                _write_location += 1;
            } else {
                throw std::runtime_error {"emplace on full buffer"};
            }
        }

        /*
         * adds an object to the buffer if room is available via in-place
         * construction; throws an exception of type std::runtime_error  if no
         * room is available in the buffer.
         */
        template <typename ... Args>
        void emplace (Args && ... args)
        {
            if (_buffered < N) {
                auto const loc {&_write_location->data[0]};
                new (loc) T {std::forward <Args> (args)...};
                _write_location += 1;
            } else {
                throw std::runtime_error {"emplace on full buffer"};
            }
        }

        /*
         * removes the first element from buffer if such an element exists, and
         * otherwise does nothing.
         */
        void pop (void) noexcept (noexcept (~T ()))
        {
            if (_buffered) {
                auto & e {_read_location [0]};
                e.~T ();
                _read_location -= 1;
                _buffered -= 1;
            }
        }
    };
} // namespace dsa

#endif // ifndef DSA_RINGBUFFER_HPP
