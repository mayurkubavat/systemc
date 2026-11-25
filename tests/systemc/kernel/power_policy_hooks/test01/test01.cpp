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

#include <systemc>
#include <iostream>

using namespace sc_core;
using namespace sc_dt;

namespace {

class test_domain_policy : public sc_domain_policy_if
{
public:
    explicit test_domain_policy( const char* nm )
      : m_name( nm ), m_corrupt( false )
    {}

    const char* domain_name() const override { return m_name; }
    bool is_power_corrupt() const override { return m_corrupt; }
    void set_corrupt( bool c ) { m_corrupt = c; }

private:
    const char* m_name;
    bool        m_corrupt;
};

class test_channel_policy : public sc_channel_power_if
{
public:
    test_channel_policy()
      : m_action( SC_PWR_NORMAL ), m_seed( 0xCAFEULL )
    {}

    void configure( sc_power_action act, uint64 seed = 0xCAFEULL )
    {
        m_action = act;
        m_seed = seed;
        set_active( act != SC_PWR_NORMAL );
    }

    sc_power_action evaluate_power_action(
        sc_signal_channel* /*channel_p*/,
        uint64&            scramble_seed_out ) override
    {
        scramble_seed_out = m_seed;
        if ( m_action == SC_PWR_RESTORE_RETAINED ) {
            const sc_power_action ret = m_action;
            m_action = SC_PWR_NORMAL;
            set_active( false );
            return ret;
        }
        return m_action;
    }

private:
    sc_power_action m_action;
    uint64          m_seed;
};

SC_MODULE( gated_rtl_block )
{
    sc_in<bool>                  clk{ "clk" };
    sc_signal<sc_uint<16>>       cfg_reg{ "cfg_reg" };
    sc_signal<sc_uint<16>>       cnt_reg{ "cnt_reg" };
    sc_signal<sc_uint<16>>       comb_out{ "comb_out" };
    sc_signal<bool>              irq_out{ "irq_out" };
    sc_signal<sc_lv<8>>          bus_lv{ "bus_lv" };
    sc_signal_rv<8>              tri_rv{ "tri_rv" };

    sc_process_handle seq_h;
    sc_process_handle comb_h;

    SC_CTOR( gated_rtl_block )
    {
        SC_METHOD( seq_proc );
        sensitive << clk.pos();
        dont_initialize();
        seq_h = sc_get_current_process_handle();

        SC_METHOD( comb_proc );
        sensitive << cfg_reg << cnt_reg;
        comb_h = sc_get_current_process_handle();
    }

    void seq_proc()
    {
        if ( cfg_reg.read() == 0 ) {
            cfg_reg.write( 0x1234 );
        }
        cnt_reg.write( cnt_reg.read() + 1 );
        irq_out.write( true );
        bus_lv.write( "10100101" );
        tri_rv.write( "11001100" );
    }

    void comb_proc()
    {
        comb_out.write( cfg_reg.read() ^ cnt_reg.read() );
    }
};

SC_MODULE( edge_watcher )
{
    sc_in<bool> sig{ "sig" };
    int pos_cnt = 0;
    int neg_cnt = 0;

    SC_CTOR( edge_watcher )
    {
        SC_METHOD( on_pos );
        sensitive << sig.pos();
        dont_initialize();

        SC_METHOD( on_neg );
        sensitive << sig.neg();
        dont_initialize();
    }

    void on_pos() { ++pos_cnt; }
    void on_neg() { ++neg_cnt; }
};

} // namespace

int sc_main( int, char*[] )
{
    sc_clock clk( "clk", 10, SC_NS );
    gated_rtl_block dut( "dut" );
    dut.clk( clk );

    edge_watcher watcher( "watcher" );
    watcher.sig( dut.irq_out );

    test_domain_policy dom( "PD_DUT" );
    dut.set_power_domain_policy( &dom );
    sc_assert( dut.get_power_domain_policy() == &dom );

    test_channel_policy cfg_pol, cnt_pol, irq_pol, lv_pol, rv_pol, comb_pol;
    dut.cfg_reg.set_power_policy( &cfg_pol );
    dut.cnt_reg.set_power_policy( &cnt_pol );
    dut.irq_out.set_power_policy( &irq_pol );
    dut.bus_lv.set_power_policy( &lv_pol );
    dut.tri_rv.set_power_policy( &rv_pol );
    dut.comb_out.set_power_policy( &comb_pol );

    // 1. Run 3 active clock cycles (25 ns: posedges at 0, 10, 20 ns)
    sc_start( 25, SC_NS );
    sc_assert( dut.seq_h.has_edge_sensitivity() );
    sc_assert( !dut.comb_h.has_edge_sensitivity() );
    sc_assert( dut.comb_out.power_last_writer() != nullptr );
    sc_assert( !dut.comb_out.power_last_writer()->has_edge_sensitivity() );
    sc_assert( dut.cnt_reg.power_last_writer() != nullptr );
    sc_assert( dut.cnt_reg.power_last_writer()->has_edge_sensitivity() );

    std::cout << "active: cfg=0x" << std::hex << dut.cfg_reg.read().to_uint()
              << " cnt=" << std::dec << dut.cnt_reg.read().to_uint()
              << " comb=0x" << std::hex << dut.comb_out.read().to_uint()
              << " irq=" << std::dec << dut.irq_out.read()
              << " lv=" << dut.bus_lv.read()
              << " rv=" << dut.tri_rv.read() << std::endl;

    // 2. Save cfg_reg, isolate irq_out to 0 and tri_rv to Z, corrupt domain, and gate processes
    dut.cfg_reg.power_capture_retention();
    irq_pol.configure( SC_PWR_CLAMP_ZERO );
    rv_pol.configure( SC_PWR_CLAMP_Z );
    cfg_pol.configure( SC_PWR_CORRUPT_FLIP );
    cnt_pol.configure( SC_PWR_CORRUPT_FLIP );
    lv_pol.configure( SC_PWR_CORRUPT_FLIP );
    dom.set_corrupt( true );

    dut.seq_h.set_power_gated( true );
    dut.comb_h.set_power_gated( true );

    dut.cfg_reg.trigger_power_update();
    dut.cnt_reg.trigger_power_update();
    dut.irq_out.trigger_power_update();
    dut.bus_lv.trigger_power_update();
    dut.tri_rv.trigger_power_update();

    // Advance 30 ns while OFF (posedges at 30, 40, 50 ns): seq_proc must NOT run
    sc_start( 30, SC_NS );
    std::cout << "off: cfg=0x" << std::hex << dut.cfg_reg.read().to_uint()
              << " cnt=0x" << dut.cnt_reg.read().to_uint()
              << " irq=" << std::dec << dut.irq_out.read()
              << " lv=" << dut.bus_lv.read()
              << " rv=" << dut.tri_rv.read()
              << " neg_cnt=" << watcher.neg_cnt << std::endl;

    // 3. Power-up at 55 ns: clear cnt_reg (POR), restore cfg_reg, ungate processes, release isolation
    dom.set_corrupt( false );
    cfg_pol.configure( SC_PWR_RESTORE_RETAINED );
    cnt_pol.configure( SC_PWR_CORRUPT_ZERO );
    lv_pol.configure( SC_PWR_NORMAL );
    rv_pol.configure( SC_PWR_NORMAL );
    dut.cfg_reg.trigger_power_update();
    dut.cnt_reg.trigger_power_update();
    dut.tri_rv.trigger_power_update();
    sc_start( SC_ZERO_TIME );

    cnt_pol.configure( SC_PWR_NORMAL );
    irq_pol.configure( SC_PWR_NORMAL );
    dut.seq_h.set_power_gated( false );
    dut.comb_h.set_power_gated( false );
    sc_start( SC_ZERO_TIME );

    std::cout << "restored: cfg=0x" << std::hex << dut.cfg_reg.read().to_uint()
              << " cnt=" << std::dec << dut.cnt_reg.read().to_uint()
              << " comb=0x" << std::hex << dut.comb_out.read().to_uint() << std::endl;

    // 4. Advance 10 ns (posedge at 60 ns) to confirm normal operation resumes
    sc_start( 10, SC_NS );
    std::cout << "resumed: cfg=0x" << std::hex << dut.cfg_reg.read().to_uint()
              << " cnt=" << std::dec << dut.cnt_reg.read().to_uint()
              << " comb=0x" << std::hex << dut.comb_out.read().to_uint()
              << " irq=" << std::dec << dut.irq_out.read()
              << " rv=" << dut.tri_rv.read()
              << " pos_cnt=" << watcher.pos_cnt << std::endl;

    // 5. Verify retention corruption helper
    dut.cnt_reg.power_capture_retention();
    dut.cnt_reg.power_corrupt_retention( 0x1234ULL );
    cnt_pol.configure( SC_PWR_RESTORE_RETAINED );
    dut.cnt_reg.trigger_power_update();
    sc_start( SC_ZERO_TIME );
    sc_assert( dut.cnt_reg.read().to_uint() != 1 );

    return 0;
}
