# MMPTCP: A Multi-Path Transport Protocol for Data Centers
Modern data centres provide large aggregate network capacity and
multiple paths among servers. Traffic is very diverse; most of the
data is produced by long, bandwidth hungry flows but the large
majority of flows, which commonly come with strict deadlines regarding
their completion time, are short. It has been shown that TCP is not
efficient for any of these types of traffic in modern data
centres. More recent protocols such MultiPath TCP (MPTCP) are very
efficient for long flows, but are ill-suited for short flows.

In this paper, we present MMPTCP, a novel transport protocol which,
compared to TCP and MPTCP, reduces short flows' completion time, while
providing excellent goodput to long flows. To do so, MMPTCP runs in
two phases; initially, it randomly scatters packets in the network
under a single congestion window exploiting all available paths. This
is beneficial to latency-sensitive flows. After a specific amount of
data is sent, MMPTCP switches to a regular MultiPath TCP mode. MMPTCP
is incrementally deployable in existing data centres as it does not
require any modifications outside the transport layer and behaves well
when competing with legacy TCP and MPTCP flows. Our extensive
experimental evaluation in simulated FatTree topologies shows that all
design objectives for MMPTCP are met.

The full paper is available [here](https://ieeexplore.ieee.org/document/7524530).

# Installations
We have tested this repository on Mac (with llvm-gcc42 and python 2.7.3-11)
and several Linux distributions (e.g. Red Hat with gcc4.4.7 or
Ubuntu16.4 with gcc5.4.0). 

The following configuration enables the optimized run and disables the
python binding in a Linux machine.

1. Clone the MPTCP's repository

``` shell
git clone https://github.com/mkheirkhah/mmptcp.git
```

2. Configure and build the ns-3 simulator

``` shell
CXXFLAGS="-Wall" ./waf configure build --build-profile=optimized --disable-python 
```

3. Run a simulation

``` shell
./waf --run "FatTree"
```

# Simulations

All simulations presented in the MMPTCP paper are conducted in a FatTree
topology. The simulation script is available in the [scratch folder](./scratch/).

# Contact

``` shell
Morteza Kheirkhah, University College London (UCL), m.kheirkhah@ucl.ac.uk
```

# Reference

The full bibtex is available at the [IEEExplore Digital
Library](https://ieeexplore.ieee.org/document/7524530). 

``` text
@inproceedings{kheirkhah2016mmptcp,
  title={{MMPTCP: A multipath transport protocol for data centers}},
  author={Kheirkhah, Morteza and Wakeman, Ian and Parisis, George},
  booktitle={IEEE INFOCOM 2016 - The 35th Annual IEEE International Conference on Computer Communications},
  pages={1-9},
  month={April},
  year={2016},
  doi={10.1109/INFOCOM.2016.7524530}
}
```

# Conditions to use this source code

* If you use the MMPTCP source code in part or entirely, please
  consider citing the MMPTCP paper.
* If you want to release your code, which is built atop our source
  code, please consider releasing a patch.
