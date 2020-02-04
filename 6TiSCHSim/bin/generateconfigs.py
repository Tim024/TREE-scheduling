
for n in [25]:
    for sched in ['MSF','TREE']:
        configfile = '''
{
    "version":                                             0,
    "execution": {
        "numCPUs":                                         7,
        "numRuns":                                         32
    },
    "settings": {
        "combination": {
            "exec_numMotes":                               ['''+str(n)+''']
        },
        "regular": {
            "exec_numSlotframesPerRun":                    4000,
            "exec_minutesPerRun":                          null,
            "exec_randomSeed":                             "random",

            "secjoin_enabled":                             false,

            "app":                                         "AppCustom",
            "app_pkPeriod":                                20,
            "app_pkPeriodVar":                             0.05,
            "app_pkLength":                                90,
            "app_burstTimestamp":                          null,
            "app_burstNumPackets":                         0,

            "rpl_of":                                      "OFStaticLinkPDR",
            "rpl_daoPeriod":                               60,
            "rpl_extensions":                              ["dis_unicast"],

            "fragmentation":                               "FragmentForwarding",
            "sixlowpan_reassembly_buffers_num":            1,
            "fragmentation_ff_discard_vrb_entry_policy":   [],
            "fragmentation_ff_vrb_table_size":             50,
            "tsch_max_payload_len":                        90,

            "sf_class":                                    "'''+str(sched)+'''",

            "tsch_slotDuration":                           0.010,
            "tsch_slotframeLength":                        101,
            "tsch_probBcast_ebProb":                       0.3,
            "tsch_clock_max_drift_ppm":                    0,
            "tsch_clock_frequency":                        32768,
            "tsch_keep_alive_interval":                    0,
            "tsch_tx_queue_size":                          32,
            "tsch_max_tx_retries":                         8,


            "radio_stats_log_period_s":                    60,

            "conn_class":                                  "C'''+str(n)+'''",
            "conn_simulate_ack_drop":                      false,

            "conn_trace":                                  null,

            "conn_random_square_side":                     2.000,
            "conn_random_init_min_pdr":                    0.5,
            "conn_random_init_min_neighbors":              3,

            "phy_numChans":                                16,

            "motes_eui64":                                 []
        }
    },
    "logging":                                             "all",
    "log_directory_name":                                  "startTime",
    "post": [
        "python compute_kpis_TREE.py"
    ]
}
        '''
        f = open('./configs/config'+str(sched)+str(n)+'.json','w')
        f.write(configfile)
        f.close()

        print('''
python runSim.py --config='configs/config'''+str(sched)+str(n)+'''.json'
sleep 2
mv $(ls -td -- ./simData/*/ | head -n1) ./simData/C'''+str(n)+'''_'''+str(sched)+'''_p10
        ''')