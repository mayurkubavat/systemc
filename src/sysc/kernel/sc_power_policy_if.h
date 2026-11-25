/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_power_policy_if.h -- On-demand kernel policy interfaces for low-power
                          simulation (domain gating, channel corruption,
                          isolation clamping, and state retention).

 *****************************************************************************/

#ifndef SC_POWER_POLICY_IF_H_INCLUDED_
#define SC_POWER_POLICY_IF_H_INCLUDED_

#include "sysc/kernel/sc_cmnhdr.h"
#include "sysc/datatypes/bit/sc_logic.h"
#include "sysc/datatypes/int/sc_nbdefs.h"

#include <cstdint>
#include <type_traits>

namespace sc_dt {
class sc_bv_base;
class sc_lv_base;
class sc_int_base;
class sc_uint_base;
class sc_signed;
class sc_unsigned;
} // namespace sc_dt

namespace sc_core {

class sc_module;
class sc_process_b;
class sc_signal_channel;

enum sc_power_action {
    SC_PWR_NORMAL = 0,
    SC_PWR_CLAMP_ZERO,
    SC_PWR_CLAMP_ONE,
    SC_PWR_CLAMP_Z,
    SC_PWR_CLAMP_LATCH,
    SC_PWR_CORRUPT_FLIP,
    SC_PWR_CORRUPT_SCRAMBLE,
    SC_PWR_CORRUPT_ZERO,
    SC_PWR_CORRUPT_ONE,
    SC_PWR_RESTORE_RETAINED
};

// ----------------------------------------------------------------------------
//  CLASS : sc_domain_policy_if
//
//  Optional power-domain policy attached on demand to an sc_module scope.
// ----------------------------------------------------------------------------

class SC_API sc_domain_policy_if
{
public:
    virtual ~sc_domain_policy_if() = default;

    virtual const char* domain_name() const = 0;
    virtual bool is_power_corrupt() const = 0;
};

// ----------------------------------------------------------------------------
//  CLASS : sc_channel_power_if
//
//  Optional power-intent policy attached on demand to an sc_signal_channel.
// ----------------------------------------------------------------------------

class SC_API sc_channel_power_if
{
public:
    sc_channel_power_if() : m_active( false ) {}
    virtual ~sc_channel_power_if() = default;

    bool is_active() const { return m_active; }
    void set_active( bool active ) { m_active = active; }

    virtual sc_power_action evaluate_power_action(
        sc_signal_channel* channel_p,
        sc_dt::uint64&     scramble_seed_out ) = 0;

    virtual sc_power_action evaluate_power_action(
        sc_signal_channel* channel_p,
        sc_dt::uint64&     scramble_seed_out,
        bool               /*value_changing*/ )
    {
        return evaluate_power_action( channel_p, scramble_seed_out );
    }

    virtual sc_power_action evaluate_driver_power_action(
        sc_signal_channel* /*channel_p*/,
        sc_process_b*      /*driver_proc_p*/,
        sc_dt::uint64&     /*scramble_seed_out*/ )
    {
        return SC_PWR_NORMAL;
    }

private:
    bool m_active;
};

// ----------------------------------------------------------------------------
//  INTERNAL : Type-aware value manipulation helpers for sc_signal_t<T, POL>
// ----------------------------------------------------------------------------

namespace sc_power_detail {

inline sc_dt::uint64 splitmix64( sc_dt::uint64 x )
{
    x += 0x9e3779b97f4a7c15ULL;
    x = ( x ^ ( x >> 30 ) ) * 0xbf58476d1ce4e5b9ULL;
    x = ( x ^ ( x >> 27 ) ) * 0x94d049bb133111ebULL;
    return x ^ ( x >> 31 );
}

std::true_type  is_lv_ptr( const sc_dt::sc_lv_base* );
std::false_type is_lv_ptr( const void* );

std::true_type  is_bv_ptr( const sc_dt::sc_bv_base* );
std::false_type is_bv_ptr( const void* );

std::true_type  is_uint_base_ptr( const sc_dt::sc_uint_base* );
std::false_type is_uint_base_ptr( const void* );

std::true_type  is_int_base_ptr( const sc_dt::sc_int_base* );
std::false_type is_int_base_ptr( const void* );

std::true_type  is_unsigned_ptr( const sc_dt::sc_unsigned* );
std::false_type is_unsigned_ptr( const void* );

std::true_type  is_signed_ptr( const sc_dt::sc_signed* );
std::false_type is_signed_ptr( const void* );

template< typename T >
struct is_sc_lv : decltype( is_lv_ptr( static_cast<const T*>( nullptr ) ) ) {};

template< typename T >
struct is_sc_bv : decltype( is_bv_ptr( static_cast<const T*>( nullptr ) ) ) {};

template< typename T >
struct is_sc_int_or_uint : std::integral_constant<bool,
    decltype( is_uint_base_ptr( static_cast<const T*>( nullptr ) ) )::value ||
    decltype( is_int_base_ptr( static_cast<const T*>( nullptr ) ) )::value ||
    decltype( is_unsigned_ptr( static_cast<const T*>( nullptr ) ) )::value ||
    decltype( is_signed_ptr( static_cast<const T*>( nullptr ) ) )::value> {};

template< typename T >
void apply_action_value(
    sc_power_action act,
    T&              dst,
    const T&        cur,
    const T&        retained,
    const T&        latched,
    sc_dt::uint64   seed )
{
    switch ( act ) {
    case SC_PWR_NORMAL:
        return;

    case SC_PWR_RESTORE_RETAINED:
        dst = retained;
        return;

    case SC_PWR_CLAMP_LATCH:
        dst = latched;
        return;

    case SC_PWR_CLAMP_Z:
        if constexpr ( std::is_same_v<T, sc_dt::sc_logic> ) {
            dst = sc_dt::SC_LOGIC_Z;
        } else if constexpr ( is_sc_lv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, sc_dt::Log_Z );
            }
        } else if constexpr ( std::is_same_v<T, bool> ) {
            dst = false;
        } else if constexpr ( is_sc_bv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, false );
            }
        } else if constexpr ( is_sc_int_or_uint<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst[i] = 0;
            }
        } else if constexpr ( std::is_integral_v<T> ) {
            dst = static_cast<T>( 0 );
        } else {
            dst = T();
        }
        return;

    case SC_PWR_CLAMP_ZERO:
    case SC_PWR_CORRUPT_ZERO:
        if constexpr ( std::is_same_v<T, bool> ) {
            dst = false;
        } else if constexpr ( std::is_same_v<T, sc_dt::sc_logic> ) {
            dst = sc_dt::SC_LOGIC_0;
        } else if constexpr ( is_sc_lv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, sc_dt::Log_0 );
            }
        } else if constexpr ( is_sc_bv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, false );
            }
        } else if constexpr ( is_sc_int_or_uint<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst[i] = 0;
            }
        } else if constexpr ( std::is_integral_v<T> ) {
            dst = static_cast<T>( 0 );
        } else {
            dst = T();
        }
        return;

    case SC_PWR_CLAMP_ONE:
    case SC_PWR_CORRUPT_ONE:
        if constexpr ( std::is_same_v<T, bool> ) {
            dst = true;
        } else if constexpr ( std::is_same_v<T, sc_dt::sc_logic> ) {
            dst = sc_dt::SC_LOGIC_1;
        } else if constexpr ( is_sc_lv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, sc_dt::Log_1 );
            }
        } else if constexpr ( is_sc_bv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, true );
            }
        } else if constexpr ( is_sc_int_or_uint<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst[i] = 1;
            }
        } else if constexpr ( std::is_integral_v<T> ) {
            using U = typename std::make_unsigned<T>::type;
            dst = static_cast<T>( ~U( 0 ) );
        } else {
            dst = T();
        }
        return;

    case SC_PWR_CORRUPT_FLIP:
        if constexpr ( std::is_same_v<T, bool> ) {
            dst = !cur;
        } else if constexpr ( std::is_same_v<T, sc_dt::sc_logic> ) {
            dst = sc_dt::SC_LOGIC_X;
        } else if constexpr ( is_sc_lv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, sc_dt::Log_X );
            }
        } else if constexpr ( is_sc_bv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, !cur.get_bit( i ) );
            }
        } else if constexpr ( is_sc_int_or_uint<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst[i] = !cur[i].to_bool();
            }
        } else if constexpr ( std::is_integral_v<T> ) {
            using U = typename std::make_unsigned<T>::type;
            dst = static_cast<T>( ~static_cast<U>( cur ) );
        } else {
            dst = T();
        }
        return;

    case SC_PWR_CORRUPT_SCRAMBLE: {
        sc_dt::uint64 word = splitmix64( seed );
        if constexpr ( std::is_same_v<T, bool> ) {
            dst = static_cast<bool>( word & 1ULL );
        } else if constexpr ( std::is_same_v<T, sc_dt::sc_logic> ) {
            dst = sc_dt::SC_LOGIC_X;
        } else if constexpr ( is_sc_lv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                dst.set_bit( i, sc_dt::Log_X );
            }
        } else if constexpr ( is_sc_bv<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                if ( i > 0 && ( i & 63 ) == 0 ) {
                    word = splitmix64( word );
                }
                dst.set_bit( i, static_cast<bool>( ( word >> ( i & 63 ) ) & 1ULL ) );
            }
        } else if constexpr ( is_sc_int_or_uint<T>::value ) {
            const int len = dst.length();
            for ( int i = 0; i < len; ++i ) {
                if ( i > 0 && ( i & 63 ) == 0 ) {
                    word = splitmix64( word );
                }
                dst[i] = static_cast<int>( ( word >> ( i & 63 ) ) & 1ULL );
            }
        } else if constexpr ( std::is_integral_v<T> ) {
            using U = typename std::make_unsigned<T>::type;
            dst = static_cast<T>( static_cast<U>( word ) );
        } else {
            dst = T();
        }
        return;
    }
    }
}

class SC_API sc_signal_power_ext_base
{
public:
    explicit sc_signal_power_ext_base( sc_channel_power_if* policy_p )
      : m_policy_p( policy_p )
      , m_last_writer_p( nullptr )
    {}
    virtual ~sc_signal_power_ext_base() = default;

    sc_channel_power_if* policy() const { return m_policy_p; }
    void set_policy( sc_channel_power_if* policy_p ) { m_policy_p = policy_p; }

    sc_process_b* last_writer() const { return m_last_writer_p; }
    void set_last_writer( sc_process_b* writer_p )
        { if ( writer_p != nullptr ) m_last_writer_p = writer_p; }

    bool is_active() const
        { return m_policy_p != nullptr && m_policy_p->is_active(); }

private:
    sc_channel_power_if* m_policy_p;
    sc_process_b*        m_last_writer_p;
};

template< typename T >
class sc_signal_power_ext : public sc_signal_power_ext_base
{
public:
    sc_signal_power_ext( sc_channel_power_if* policy_p, const T& init_val )
      : sc_signal_power_ext_base( policy_p )
      , m_retained_val( init_val )
      , m_latched_val( init_val )
    {}

    T m_retained_val;
    T m_latched_val;
};

} // namespace sc_power_detail

} // namespace sc_core

#endif /* SC_POWER_POLICY_IF_H_INCLUDED_ */
