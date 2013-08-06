require("data.table")

approaches <- c("pure_ff", "inc_robust")
domains <- c("zenotravel", "depots")

# Get list of incomplete domains for a STRIPS domain
# Example: zenotravel --> "zenotravel.pddl.0p_0a_1d.0" etc
get_incomplete_domains <- function(domain) {
  num_ppres <- 6
  num_padds <- 6
  num_pdels <- 6
  
  prefix <- paste(domain,"pddl",sep=".")
  result <- rep(prefix, num_ppres * num_padds * num_pdels - 1)
  index <- 1
  for (p in 0:(num_ppres-1))
    for (a in 0:(num_padds-1))
      for (d in 0:(num_pdels-1)) {
        if (p + a + d > 0) {
          s <- paste(as.character(p),"p", sep="");
          s <- paste(s, paste(as.character(a),"a", sep=""), sep="_");
          s <- paste(s, paste(as.character(d),"d", sep=""), sep="_");
          result[index] <- paste(result[index], s, sep=".")
          result[index] <- paste(result[index], "0", sep=".")
          index <- index + 1
        }
      }
  return (result)
}

get_problems <- function() {
  num_probs <- 10
  prefix <- "pfile"
  result <- rep(prefix, num_probs)
  for (i in 1:num_probs) {
    result[i] <- paste(result[i],as.character(i),sep="")
  }
  return (result)
}

zenotravel_lower_pure_ff <- data.table(read.csv("zenotravel@lower@pure_ff.csv", comment.char='#'))
#driverlog_lower_pure_ff <- data.table(read.csv("driverlog@lower@pure_ff.csv", comment.char='#'))
depots_lower_pure_ff <- data.table(read.csv("depots@lower@pure_ff.csv", comment.char='#'))
rover_lower_pure_ff <- data.table(read.csv("rover@lower@pure_ff.csv", comment.char='#'))

zenotravel_upper_pure_ff <- data.table(read.csv("zenotravel@upper@pure_ff.csv", comment.char='#'))
#driverlog_upper_pure_ff <- data.table(read.csv("driverlog@upper@pure_ff.csv", comment.char='#'))
depots_upper_pure_ff <- data.table(read.csv("depots@upper@pure_ff.csv", comment.char='#'))
rover_upper_pure_ff <- data.table(read.csv("rover@upper@pure_ff.csv", comment.char='#'))

zenotravel_lower_inc_robust <- data.table(read.csv("zenotravel@lower@inc_robust.csv", comment.char='#'))
driverlog_lower_inc_robust <- data.table(read.csv("driverlog@lower@inc_robust.csv", comment.char='#'))
depots_lower_inc_robust <- data.table(read.csv("depots@lower@inc_robust.csv", comment.char='#'))
rover_lower_inc_robust <- data.table(read.csv("rover@lower@inc_robust.csv", comment.char='#'))

zenotravel_upper_inc_robust <- data.table(read.csv("zenotravel@upper@inc_robust.csv", comment.char='#'))
driverlog_upper_inc_robust <- data.table(read.csv("driverlog@upper@inc_robust.csv", comment.char='#'))
depots_upper_inc_robust <- data.table(read.csv("depots@upper@inc_robust.csv", comment.char='#'))
rover_upper_inc_robust <- data.table(read.csv("rover@upper@inc_robust.csv", comment.char='#'))

# Compare total number of plans returned
plans_comparison <- function() {
  
  
  
}


