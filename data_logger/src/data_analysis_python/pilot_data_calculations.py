online_omni_t = 33.33
offline_omni_t = 75
online_pendant_t = 40
offline_pendant_t = 120

num_models = 3
num_positions = 2
num_trials = 3

min_refinements = 1
max_refinements = 3

min_tot_refinements = num_models * num_positions * num_trials * min_refinements
max_tot_refinements = num_models * num_positions * num_trials * max_refinements

print("Minimum number of refinemetns = " + str(min_tot_refinements))
print("Maximum number of refinemetns = " + str(max_tot_refinements))

min_time = ( online_omni_t + offline_omni_t + online_pendant_t + offline_pendant_t ) * min_tot_refinements / (60 * 60)
max_time = ( online_omni_t + offline_omni_t + online_pendant_t + offline_pendant_t ) * max_tot_refinements / (60 * 60)

print("Minimum active experiment time = " + str(min_time) + " hours")
print("Maximum active experiment time = " + str(max_time) + " hours")

# total_active_ratio = 60/18
total_active_ratio = 1.5

print("Estimated minimum total experiment time = " + str(min_time * total_active_ratio) + " hours")
print("Estimated maximum total experiment time = " + str(max_time * total_active_ratio) + " hours")
