#!/bin/sh

num_output_domains=5
num_poss_pres=5
num_poss_adds=5
num_poss_dels=5

#./DomainGenerator -P ./depots/ -i depots.pddl -n $num_output_domains -p $num_poss_pres -a $num_poss_adds -d $num_poss_dels

#./DomainGenerator -P ./driverlog/ -i driverlog.pddl -n $num_output_domains -p $num_poss_pres -a $num_poss_adds -d $num_poss_dels

#./DomainGenerator -P ./freecell/ -i freecell.pddl -n $num_output_domains -p $num_poss_pres -a $num_poss_adds -d $num_poss_dels

#./DomainGenerator -P ./rover/ -i rover.pddl -n $num_output_domains -p $num_poss_pres -a $num_poss_adds -d $num_poss_dels

./DomainGenerator -P ./satellite/ -i satellite.pddl -n $num_output_domains -p $num_poss_pres -a $num_poss_adds -d $num_poss_dels

#./DomainGenerator -P ./zenotravel/ -i zenotravel.pddl -n $num_output_domains -p $num_poss_pres -a $num_poss_adds -d $num_poss_dels


