wynglasAnneal_sweep() {
    julia160rc1 -O3 ParamSweep.jl $1 --anneal --spreadNH_start $2 --spreadNH_step $3 --spreadNH_stop $4  --inflation_start $2 --inflation_step $3 --inflation_stop $4
}

wynglasAnneal_30_05_05() {
    wynglasAnneal_sweep $1 3.0 -0.5 0.5
}

wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-10" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-9" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-8" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-7" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-6" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-5" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-4" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-3" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-2" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_-1" &
sleep 10
wynglasAnneal_30_05_05 "skuif_X_l7_8_init/fg_0"
