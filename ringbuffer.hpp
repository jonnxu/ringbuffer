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

#include <array>        // std::array
#include <type_traits>  // std::remove_cv, std::is_nothrow_move_assignable,
                        // std::is_nothrow_copy_assignable,
                        // std::is_trivially_destructible
#include <utility>      // std::forward, std::move, std::swap


namespace dsa
{
    /*
     *  Description
     *  -----------
     *
     *  dsa::ringbuffer <> is an implementation of an STL-style circular buffer.
     *
     *  The size of the buffer is fixed by the template parameter N. Please see
     *  https://github.com/daltonwoodard/dynamic_ringbuffer.git for a resizeable
     *  version.
     *
     *  This implementation is NOT threadsafe. Please see
     *  https://github.com/daltonwoodard/atomic_ringbuffer.git for a thread-safe
     *  version implemented with C++ atomic primitives.
     *
     *  If and only if the stored object type T has strong exception safe
     *  constructors is the following guaranteed:
     *
     *  If this object is successfull constructed, then throughout the
     *  object's lifetime:
     *
     *      i.  all view operations (front/back) are guaranteed as nothrow.
     *      ii. read opeartions provide the basic exception safety guarantee;
     *
     *  Notice that the read operations modify the data structure (by removing
     *  elements from the front); hence they are, in actuallity, writes.
     *
     *  Template Parameters
     *  -------------------
     *  - T: the object type to be buffered. This type does *not* have to be
     *  default constructable.
     *
     *  - N: the maximum number of elements for the buffer to hold; the number N
     *  must be nonzero.
     *
     *  Member Types
     *  ------------
     *  - value_type:      std::remove_cv <T>::type;
     *  - size_type:       std::size_t;
     *  - difference_type: std::ptrdiff_t;
     *  - pointer:         value_type *;
     *  - const_pointer:   value_type const *;
     *  - reference:       value_type &;
     *  - const_reference: value_type const &;
     *
     *  - iterator:               models RandomAccessIterator
     *  - const_iterator:         models RandomAccessIterator
     *  - reverse_iterator:       std::reverse_iterator <iterator>;
     *  - const_reverse_iterator: std::reverse_iterator <const_iterator>;
     *
     *  Member Functions
     *  ----------------
     *  - front: access the first element
     *  - back:  access the last element
     *
     *  - empty: checks whether the buffer is empty
     *  - size:  returns the number of buffered elements
     *
     *  - push:    inserts an element at the end
     *  - emplace: constructs an element in-place at the end
     *  - pop:     removes the first element
     *
     *  - swap: swaps the contents. Template typename T must be Swappable.
     */
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

        template <typename U, std::size_t BuffSize>
        class iterator_impl;

        iterator_impl <T, N> _write_location {
            reinterpret_cast <unqualified_pointer> (&_buffer [0]),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last)
        };

        iterator_impl <T, N> _read_location  {
            reinterpret_cast <unqualified_pointer> (&_buffer [0]),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last)
        };

        template <typename U, std::size_t BuffSize>
        class iterator_impl : public std::iterator <
            std::random_access_iterator_tag, U, std::ptrdiff_t, U *, U &
        >
        {
        private:
            using iter_type = std::iterator <
                std::random_access_iterator_tag, U, std::ptrdiff_t, U *, U &
            >;

            U * _iter;
            U * const _first;
            U * const _last;

        public:
            using difference_type   = typename iter_type::difference_type;
            using value_type        = typename iter_type::value_type;
            using pointer           = typename iter_type::pointer;
            using const_pointer     = typename iter_type::const_pointer;
            using reference         = typename iter_type::reference;
            using const_reference   = typename iter_type::const_reference;
            using iterator_category = typename iter_type::iterator_category;

            iterator_impl (void) = delete;

            iterator_impl (pointer p, pointer first, pointer last)
                noexcept
                : _iter  {p}
                , _first {first}
                , _last  {last}
            {}

            void swap (iterator_impl & other) noexcept
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
                /* normal configuration -- non-wraparound case */
                if (_first < _last) {
                    return this->_iter - rhs._iter;
                /*
                 * _last is behind _first (in the address space) --
                 * wraparound case, so the space from _last to _first
                 * is uninitialized.
                 */
                } else {
                    /*
                     * three cases:
                     *  i.    both iters are above _first
                     *  ii.   both iters are below _last
                     *  iii.  iters are split above _first and below _last
                     *      a. this is above _first and rhs is below _last
                     *      b. rhs is above _first and this is below _last
                     */
                    if (_first <= this->_iter && _first <= rhs._iter) {
                        return this->_iter - rhs._iter;
                    } else if (this->_iter <= _last && rhs._iter <= _last) {
                        return this->_iter - rhs._iter;
                    } else if (_first <= this->_iter && rhs._iter <= _last) {
                        return (this->_iter - rhs._iter) -
                            static_cast <difference_type> (BuffSize);
                    /* if (_first <= rhs._iter && this->_iter <= _last) */
                    } else {
                        return static_cast <difference_type> (BuffSize) -
                            (rhs._iter - this->_iter);
                    }
                }
            }

            bool operator== (iterator_impl const & rhs) const
            {
                return this->_iter == rhs._iter;
            }

            bool operator!= (iterator_impl const & rhs) const
            {
                return this->_iter != rhs._iter;
            }

            bool operator< (iterator_impl const & rhs) const
            {
                /* locigal: return this->_iter < rhs._iter; */
                return rhs - *this > 0;
            }

            bool operator> (iterator_impl const & rhs) const
            {
                /* locigal: return _iter > rhs._iter; */
                return *this - rhs > 0;
            }

            bool operator<= (iterator_impl const & rhs) const
            {
                /* locigal: return _iter <= rhs._iter; */
                if (*this == rhs) {
                    return true;
                } else {
                    return *this < rhs;
                }
            }

            bool operator>= (iterator_impl const & rhs) const
            {
                /* locigal: return _iter >= rhs._iter; */
                if (*this == rhs) {
                    return true;
                } else {
                    return *this > rhs;
                }
            }

            reference operator* (void) const
            {
                return *_iter;
            }

            reference operator[] (difference_type n) const
            {
                return *(*this + n);
            }

            pointer addressof (void) const
            {
                return _iter;
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

        using iterator        = iterator_impl <T, N>;
        using const_iterator  = iterator_impl <T const, N>;
        using reverse_iterator       = std::reverse_iterator <iterator>;
        using const_reverse_iterator = std::reverse_iterator <const_iterator>;

        ringbuffer (void) noexcept
            : _buffer   {}
            , _buffered {0}
        {}

        ringbuffer (ringbuffer const & other)
            noexcept (std::is_nothrow_copy_assignable <T>::value)
            : _buffer   {}
            , _buffered {other._buffered}
        {
            auto ti = this->_write_location;
            auto oi = other.cbegin ();
            while (oi != other.cend ()) {
                *ti = *oi;
                ti += 1;
                oi += 1;
            }

            this->_write_location += this->_buffered;
        }

        ringbuffer (ringbuffer && other)
            noexcept (std::is_nothrow_move_assignable <T>::value)
            : _buffer   {}
            , _buffered {other._buffered}
        {
            auto ti = this->_write_location;
            auto oi = other.cbegin ();
            while (oi != other.cend ()) {
                *ti = std::move (*oi);
                ti += 1;
                oi += 1;
            }

            this->_write_location += this->_buffered;
        }

    private:
        template <
            typename U = T,
            typename std::enable_if <
                std::is_trivially_destructible <U>::value
            >::type = 0
        >
        static void destruct_element (U &) noexcept
        {
            /* no-op */
        }

        template <
            typename U = T,
            typename std::enable_if <
                not std::is_trivially_destructible <U>::value
            >::type = 0
        >
        static void destruct_element (U & u)
            noexcept (std::is_nothrow_destructible <U>::value)
        {
            u.~U ();
        }

    public:
        ~ringbuffer (void)
            noexcept (noexcept (destruct_element (std::declval <T &> ())))
        {
            for (auto it = _read_location; it != _write_location; ++it) {
                destruct_element (*it);
            }
        }

        /* swaps the contents of the buffer */
        void swap (ringbuffer & other)
            noexcept (
                std::is_nothrow_move_assignable <T>::value &&
                std::is_nothrow_move_constructible <T>::value &&
                std::is_nothrow_destructible <T>::value
            )
        {
            using std::swap;

            auto ti {this->begin ()};
            auto oi {other.begin ()};

            /*
             * past-the-end iterators in this implementation point at
             * uinitialized memory, and so once we have swapped as many
             * valid objects as possible we must revert to performiing
             * in-place construction of elements into the correct locations.
             */
            if (this->_buffered <= other._buffered) {
                while (ti != this->end ()) {
                    swap (*ti, *oi);
                    ti += 1;
                    oi += 1;
                }

                while (oi != other.end ()) {
                    auto addr {ti.addressof ()};
                    new (addr) T {std::move (*oi)};
                    destruct_element (*oi);
                    ti += 1;
                    oi += 1;
                }
            } else {
                while (oi != other.end ()) {
                    swap (*oi, *ti);
                    oi += 1;
                    ti += 1;
                }

                while (ti != this->end ()) {
                    auto addr {oi.addressof ()};
                    new (addr) T {std::move (*ti)};
                    destruct_element (*ti);
                    oi += 1;
                    ti += 1;
                }
            }

            /*
             * adjust iterators for this and other:
             * - read locations for each stay the same
             * - write locations are adjusted by the difference between
             *   buffered elements of each.
             */
            {
                auto const td {this->_write_location - this->_read_location};
                auto const od {other._write_location - other._read_location};

                this->_write_location += (od - td);
                other._write_location += (td - od);

                std::swap (this->_buffered, other._buffered);
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
