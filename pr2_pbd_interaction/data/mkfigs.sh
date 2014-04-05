# Feasibility analysis: any fixes that get 0 unreachable poses on test vs amt data
python experimentAnalysis/fa.py experimentAnalysis/log3_archives/1a.npz 1a ~/repos/hcomp2014/figures

# Feasibility analysis: any fixes that get 0 unreachable poses on test vs amt data
python experimentAnalysis/fa.py experimentAnalysis/log3_archives/2a.npz 2a ~/repos/hcomp2014/figures

# Feasibility analysis: portion of fixes that get 0 unreachable poses vs start no. unreachable poses.
python experimentAnalysis/fa.py experimentAnalysis/log3_archives/2d.npz 2d ~/repos/hcomp2014/figures

# Feasibility analysis: how well can crowd's confidence predict no. unreachable poses
python experimentAnalysis/fa.py experimentAnalysis/log3_archives/log_3.txt confheat ~/repos/hcomp2014/figures
