# Model-based Decision Making with Imagination for Autonomous Parking


This paper has been accepted by [IEEE Symposium on Intelligent Vehicle (IV) 2018](https://ieeexplore.ieee.org/xpl/conhome/8472796/proceeding).

By [Ziyue Feng](ziyue.cool), Shitao Chen, Yu Chen, Nanning Zheng.

IEEE Xplore: [Link](https://ieeexplore.ieee.org/document/8500700)

Arxiv: [Link](https://arxiv.org/abs/2108.11420)


![image](https://user-images.githubusercontent.com/21237230/131201981-f348f3a2-b04e-4907-b1a4-8449a95c15d3.png)

### Abstract
Autonomous parking technology is a key concept
within autonomous driving research. This paper will propose
an imaginative autonomous parking algorithm to solve issues
concerned with parking. The proposed algorithm consists of
three parts: an imaginative model for anticipating results before
parking, an improved rapid-exploring random tree (RRT) for
planning a feasible trajectory from a given start point to a
parking lot, and a path smoothing module for optimizing the
efficiency of parking tasks. Our algorithm is based on a real
kinematic vehicle model; which makes it more suitable for
algorithm application on real autonomous cars. Furthermore,
due to the introduction of the imagination mechanism, the
processing speed of our algorithm is ten times faster than that
of traditional methods, permitting the realization of real-time
planning simultaneously. In order to evaluate the algorithm’s
effectiveness, we have compared our algorithm with traditional
RRT ,within three different parking scenarios. Ultimately,
results show that our algorithm is more stable than traditional
RRT and performs better in terms of efficiency and quality.

### Requirements
* OpenCV
* [LEDA](https://www.algorithmic-solutions.com/index.php/products/leda-for-c)

### Simple Video Demo
[![image](https://user-images.githubusercontent.com/21237230/131202639-035fb220-ffe0-45a1-bf0d-ae86ee63186e.png)](https://www.youtube.com/watch?v=Myhnc_oz-CY)


### Citation
```
@INPROCEEDINGS{8500700,
  author={Feng, Ziyue and Chen, Shitao and Chen, Yu and Zheng, Nanning},
  booktitle={2018 IEEE Intelligent Vehicles Symposium (IV)}, 
  title={Model-Based Decision Making With Imagination for Autonomous Parking}, 
  year={2018},
  volume={},
  number={},
  pages={2216-2223},
  doi={10.1109/IVS.2018.8500700}}
```

### Reference
This project is developed from [Nonholonomic Mobile Robot Motion Planning](http://msl.cs.uiuc.edu/~lavalle/cs576_1999/projects/junqu/)

### Contact
If you have any question or concerns about this paper or repository, please feel free to contact Ziyue Feng (zfeng@clemson.edu).
