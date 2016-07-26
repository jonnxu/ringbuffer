//
// dsa is a utility library of data structures and algorithms built with C++11.
// This file (ringbuffer.hpp) is part of the dsa project.
//
// ringbuffer; a fixed-size implementation of an STL-style circular buffer for
// C++11 and later.
//
// A description of the circular buffer data structure can be found here:
//
//      https://en.wikipedia.org/wiki/Circular_buffer
//
// author: Dalton Woodard
// contact: daltonmwoodard@gmail.com
// repository: https://github.com/daltonwoodard/ringbuffer.git
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
#include <exception>    // std::logic_error
#include <type_traits>  // std::remove_cv, std::is_nothrow_move_assignable,
                        // std::is_nothrow_copy_assignable,
                        // std::is_trivially_destructible,
                        // std::is_nothrow_destructible
#include <utility>      // std::forward, std::move, std::swap


namespace dsa
{
    enum class overwrite_policy
    {
        no_overwrite,
        overwrite
    };

    /*
     *  Description
     *  -----------
     *
     *  dsa::ringbuffer <> is an implementation of an STL-style circular buffer.
     *
     *  A description of the circular buffer data structure can be found here:
     *
     *      https://en.wikipedia.org/wiki/Circular_buffer
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
     *      i.  all view operations are guaranteed as nothrow
     *      ii. write operations provide the strong exception safety guarantee
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
     *  - front:      access the first element
     *  - back:       access the last element
     *
     *  - empty: checks whether the buffer is empty
     *  - size:  returns the number of buffered elements
     *
     *  - set_overwrite_policy: sets the overwrite policy for the container
     *  - get_overwrite_policy: returns the overwrite policy for the container
     *
     *  - clear:                clears the contents of the buffer
     *  - push/push_back:       inserts an element at the end
     *  - emplace/emplace_back: constructs an element in-place at the end
     *  - pop/pop_front:        removes the first element
     *
     *  - swap: swaps the contents. Template typename T must be Swappable.
     *
     *  note:
    *       The behavior of push_back and emplace_back when capacity == 0 is
     *      determined by the current overwrite_policy of the ringbuffer
     *      [default: dsa::overwrite_policy::no_overwrite]. If the policy enum
     *      flag is equal to dsa::overwrite_policy::no_overwrite then no
     *      operation on the structure is performed and an exception of type
     *      std::logic_error is thrown. If the policy enum flag is
     *      dsa::overwrite_policy::overwrite then the first element
     *      (given by front ()) of the buffer is overwritten by the new value.
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

        enum overwrite_policy _owpolicy;

        template <typename U, std::size_t BuffSize>
        class iterator_impl;

        /*
         * The iterators _write_location and _read_location are priviledged in
         * that the logical space they work in is the whole buffer. They are
         * later used to represent the bounding logical regions when creating
         * iterators to the buffer.
         */
        iterator_impl <T, N> _write_location {
            reinterpret_cast <unqualified_pointer> (&_buffer [0]),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last)
        };

        iterator_impl <T, N> _read_location  {
            reinterpret_cast <unqualified_pointer> (&_buffer [0]),
            reinterpret_cast <unqualified_pointer> (_first),
            reinterpret_cast <unqualified_pointer> (_last),
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

            /*
             * these pointers bound the logical region with the current state of
             * the ciruclar buffer.
             */
            U * const _lfirst;
            U * const _llast;

            /* these pointers bound the address space of the backing buffer */
            U * const _rfirst;
            U * const _rlast;

            /* checks whether the logical region is actually contiguous */
            bool logical_region_is_contiguous (void) const noexcept
            {
                return _lfirst <= _llast;
            }

        public:
            using difference_type   = typename iter_type::difference_type;
            using value_type        = typename iter_type::value_type;
            using pointer           = typename iter_type::pointer;
            using reference         = typename iter_type::reference;
            using iterator_category = typename iter_type::iterator_category;

            iterator_impl (void) = delete;

            iterator_impl (pointer p,
                           pointer logical_first,
                           pointer logical_last,
                           pointer real_first,
                           pointer real_last)
                noexcept
                : _iter   {p}
                , _lfirst {logical_first}
                , _llast  {logical_last}
                , _rfirst {real_first}
                , _rlast  {real_last}
            {}

            void swap (iterator_impl & other) noexcept
            {
                std::swap (this->_iter, other._iter);
                std::swap (this->_first, other._first);
                std::swap (this->_last, other._last);
            }

            iterator_impl & operator++ (void)
            {
                if (_iter == _llast) {
                    _iter = _lfirst;
                } else {
                    _iter += 1;
                }

                return *this;
            }

            iterator_impl & operator-- (void)
            {
                if (_iter == _lfirst) {
                    _iter = _llast;
                } else {
                    _iter -= 1;
                }

                return *this;
            }

            iterator_impl operator++ (int)
            {
                auto const tmp {*this};

                if (_iter == _llast) {
                    _iter = _lfirst;
                } else {
                    _iter += 1;
                }

                return tmp;
            }

            iterator_impl operator-- (int)
            {
                auto const tmp {*this};

                if (_iter == _lfirst) {
                    _iter = _llast;
                } else {
                    _iter -= 1;
                }

                return tmp;
            }

            /*
             * the parameter n is assumed to be valid; that is, such that
             * the resulting pointer after [_iter += n] remains logically
             * bound between _lfirst and _llast.
             */
            iterator_impl & operator+= (difference_type n)
            {
                /*
                 * two cases:
                 *  i.  _lfirst <= _llast in the address space (non-wraparound)
                 *  ii. _llast < _lfirst in the address space (wraparound)
                 *      a. this is above _lfirst
                 *      b. this is below _llast
                 */

                /* case i. */
                if (this->logical_region_is_contiguous ()) {
                    _iter += n;
                /* cast ii.a. */
                } else if (_lfirst <= _iter) {
                    /* stays in-bounds */
                    if (_iter + n <= _rlast) {
                        _iter += n;
                    /* overflow past-the-end (note: n > 0) */
                    } else {
                        _iter = _rfirst + (n - 1 - (_rlast - _iter));
                    }
                /* cast ii.b. */
                } else {
                    /* stays in-bounds */
                    if (_iter + n >= _rfirst) {
                        _iter += n;
                    /* underflows before-the-beginning (note: n < 0) */
                    } else {
                        auto const m {-n};
                        _iter = _rlast - (m - 1 - (_iter - _rfirst));
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
            , _owpolicy {overwrite_policy::no_overwrite}
        {}

        ringbuffer (ringbuffer const & other)
            noexcept (std::is_nothrow_copy_assignable <T>::value)
            : _buffer   {}
            , _buffered {other._buffered}
            , _owpolicy {other._owpolicy}
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
            , _owpolicy {other._owpolicy}
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
            typename = typename std::enable_if <
                std::is_trivially_destructible <U>::value
            >::type
        >
        static void destruct_element (U &) noexcept
        {
            /* no-op */
        }

        template <
            typename U = T,
            typename = typename std::enable_if <
                not std::is_trivially_destructible <U>::value
            >::type,
            bool /* no redeclare */ = bool {}
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
            auto it {this->end () - 1};

            while (_buffered > 0) {
                destruct_element (*it);
                it -= 1;
                _buffered -= 1;
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
            /* swap elements */
            {
                using std::swap;

                auto ti {this->begin ()};
                auto oi {other.begin ()};

                auto tb {this->_buffered};
                auto ob {other._buffered};

                /*
                 * past-the-end iterators in this implementation point at
                 * uinitialized memory, and so once we have swapped as many
                 * valid objects as possible we must revert to performiing
                 * in-place construction of elements into the correct locations.
                 */
                if (tb <= ob) {
                    while (tb) {
                        swap (*ti, *oi);
                        ti += 1;
                        oi += 1;
                        tb -= 1;
                        ob -= 1;
                    }

                    while (ob) {
                        auto addr {ti.addressof ()};
                        new (addr) T {std::move (*oi)};
                        destruct_element (*oi);
                        ti += 1;
                        oi += 1;
                        ob -= 1;
                    }
                } else {
                    while (ob) {
                        swap (*oi, *ti);
                        oi += 1;
                        ti += 1;
                        ob -= 1;
                        tb -= 1;
                    }

                    while (tb) {
                        auto addr {oi.addressof ()};
                        new (addr) T {std::move (*ti)};
                        destruct_element (*ti);
                        oi += 1;
                        ti += 1;
                        tb -= 1;
                    }
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
                std::swap (this->_owpolicy, other._owpolicy);
            }
        }

        /* checks whether the buffer is emtpy */
        bool empty (void) const noexcept
        {
            return _buffered != 0;
        }

        /* returns the number of elements stored in the buffer */
        std::size_t size (void) const noexcept
        {
            return _buffered;
        }

        /* returns the number of available spaces to add elements */
        std::size_t capacity (void) const noexcept
        {
            return N - _buffered;
        }

        /* set the overwrite policy for the buffer */
        void set_overwrite_policy (enum overwrite_policy pol) noexcept
        {
            this->_owpolicy = pol;
        }

        /* returns the overwrite policy for the buffer */
        enum overwrite_policy get_overwrite_policy (void) const noexcept
        {
            return this->_owpolicy;
        }

        /* returns an iterator the start of the buffer */
        iterator begin (void) noexcept
        {
            return iterator {
                _read_location.addressof (),
                _read_location.addressof (),
                _write_location.addressof (),
                reinterpret_cast <pointer> (_first),
                reinterpret_cast <pointer> (_last)
            };
        }

        /* returns an iterator the end of the buffer */
        iterator end (void) noexcept
        {
            return iterator {
                (_write_location + 1).addressof (),
                _read_location.addressof (),
                _write_location.addressof (),
                reinterpret_cast <pointer> (_first),
                reinterpret_cast <pointer> (_last)
            };
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
            return const_iterator {
                _read_location.addressof (),
                _read_location.addressof (),
                _write_location.addressof (),
                reinterpret_cast <pointer> (_first),
                reinterpret_cast <pointer> (_last)
            };
        }

        /* returns a const iterator the end of the buffer */
        const_iterator cend (void) const noexcept
        {
            return const_iterator {
                (_write_location + 1).addressof (),
                _read_location.addressof (),
                _write_location.addressof (),
                reinterpret_cast <pointer> (_first),
                reinterpret_cast <pointer> (_last)
            };
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
            return *_read_location;
        }

        /* returns a reference to the first element in the buffer */
        const_reference front (void) const noexcept
        {
            return *_read_location;
        }

        /* returns a reference to the last element in the buffer */
        reference back (void) noexcept
        {
            if (_buffered > 0) {
                return _read_location [_buffered - 1];
            } else {
                return *_read_location;
            }
        }

        /* returns a reference to the last element in the buffer */
        const_reference back (void) const noexcept
        {
            if (_buffered > 0) {
                return _read_location [_buffered - 1];
            } else {
                return *_read_location;
            }
        }

        /*
         * clears the contents of the buffer; the elements are guaranteed to be
         * destructed in the reverse-order they were added.
         */
        void clear (void)
            noexcept (noexcept (destruct_element (std::declval <T &> ())))
        {
            auto it {this->end () - 1};

            while (_buffered > 0) {
                destruct_element (*it);
                it -= 1;
                _buffered -= 1;
            }

            _write_location = _read_location;
        }

        /*
         * Adds an object to the buffer if room is available.
         *
         * If no capacity is avaiable, then:
         *      If the overwrite policy is set to no_overwrite this method
         *      throws an exception of type std::logic_error.
         *
         *      If the overwrite policy is set to overwrite, this method
         *      overwrites the first element of the buffer.
         */
        void push (value_type const & v)
        {
            if (_buffered < N) {
                auto const addr {_write_location.addressof ()};
                new (addr) value_type {v};
                _write_location += 1;
                _buffered += 1;
            } else if (_owpolicy == overwrite_policy::overwrite) {
                auto const addr {_write_location.addressof ()};
                destruct_element (*addr);
                new (addr) value_type {v};
                _write_location += 1;
                _read_location += 1;
                _buffered = N;
            } else {
                throw std::logic_error {"push back on full buffer"};
            }
        }

        /*
         * Adds an object to the buffer if room is available.
         *
         * If no capacity is avaiable, then:
         *      If the overwrite policy is set to no_overwrite this method
         *      throws an exception of type std::logic_error.
         *
         *      If the overwrite policy is set to overwrite, this method
         *      overwrites the first element of the buffer.
         */
        void push (value_type && v)
        {
            if (_buffered < N) {
                auto const addr {_write_location.addressof ()};
                new (addr) value_type {std::move (v)};
                _write_location += 1;
                _buffered += 1;
            } else if (_owpolicy == overwrite_policy::overwrite) {
                auto const addr {_write_location.addressof ()};
                destruct_element (*addr);
                new (addr) value_type {std::move (v)};
                _write_location += 1;
                _read_location += 1;
                _buffered = N;
            } else {
                throw std::logic_error {"push back on full buffer"};
            }
        }

        void push_back (value_type const & v)
        {
            return this->push (v);
        }

        void push_back (value_type && v)
        {
            return this->push (std::move (v));
        }

        /*
         * Adds an object to the buffer if room is available using in-place
         * construction.
         *
         * If no capacity is avaiable, then:
         *      If the overwrite policy is set to no_overwrite this method
         *      throws an exception of type std::logic_error.
         *
         *      If the overwrite policy is set to overwrite, this method
         *      overwrites the first element of the buffer.
         */
        template <typename ... Args>
        void emplace (Args && ... args)
        {
            if (_buffered < N) {
                auto const addr {_write_location.addressof ()};
                new (addr) value_type {std::forward <Args> (args)...};
                _write_location += 1;
                _buffered += 1;
            } else if (_owpolicy == overwrite_policy::overwrite) {
                auto const addr {_write_location.addressof ()};
                destruct_element (*addr);
                new (addr) value_type {std::forward <Args> (args)...};
                _write_location += 1;
                _read_location += 1;
                _buffered = N;
            } else {
                throw std::logic_error {"emplace back on full buffer"};
            }
        }

        template <typename ... Args>
        void emplace_back (Args && ... args)
        {
            return this->emplace (std::forward <Args> (args)...);
        }

        /*
         * removes the first element from buffer if such an element exists, and
         * otherwise does nothing.
         */
        void pop (void) noexcept (noexcept (~T ()))
        {
            if (_buffered > 0) {
                destruct_element (*_read_location);
                _read_location += 1;
                _buffered -= 1;
            }
        }
    };
} // namespace dsa

#endif // ifndef DSA_RINGBUFFER_HPP
