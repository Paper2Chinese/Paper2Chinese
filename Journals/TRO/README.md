在这里插入图片描述
# 题目：[CMax-SLAM: Event-Based Rotational-Motion Bundle Adjustment and SLAM System Using Contrast Maximization](https://ieeexplore.ieee.org/abstract/document/10474186)  
## CMax-SLAM：基于事件的旋转运动捆集调整与SLAM系统
**作者：Shuang Guo； Guillermo Gallego** 

**源码链接：** https://github.com/tub-rip/cmax_slam
****
# 摘要
事件相机是生物启发的视觉传感器，它们捕获像素级的强度变化并输出异步事件流。它们在机器人学和计算机视觉领域，如高速和高动态范围场景中，显示出比传统相机更大的潜力。本文考虑了使用事件相机进行旋转运动估计的问题。过去十年中，已经开发了几种基于事件的旋转估计方法，但它们尚未在统一的标准下进行评估和比较。此外，这些先前的工作没有考虑全局优化步骤。为此，我们以两个目标为出发点，对这个进行了系统的研究：总结先前的工作并提出我们自己的解决方案。首先，我们从理论和实验上比较了先前的工作。其次，我们提出了第一种基于事件的仅旋转捆集调整（BA）方法。我们利用最先进的对比度最大化（CMax）框架来制定它，这是一个原理性的方法，避免了将事件转换为帧的需要。第三，我们使用提出的BA构建了CMax-SLAM，这是第一个包含前端和后端的基于事件的仅旋转SLAM系统。我们的BA能够离线（轨迹平滑）和在线（CMax-SLAM后端）运行。为了展示我们方法的性能和多功能性，我们在合成和真实世界数据集上进行了全面的实验，包括室内、室外和太空场景。我们讨论了现实世界评估的陷阱，并提出了一个代理，用于评估基于事件的旋转BA方法的重投影误差。我们发布了源代码和新的独特数据序列，以造福社区。我们希望这项工作能够促进对基于事件的自我运动估计的更好理解，并推动相关运动估计任务的研究。
# 关键词
- 计算机视觉
- 事件驱动相机
- 定位
- 建图
- SLAM
- 智能相机

# 引言

事件相机是一种新型的生物启发式视觉传感器，它们测量每个像素的亮度变化[\[1\]][\[2\]]。与传统相机产生的图像/帧不同，事件相机的输出是一系列异步事件流，$e_k = (x_k, t_k, p_k)$，包括时间戳$t_k$、像素位置$x_k = (x_k, y_k)^\top$和亮度变化的极性$p_k$。这种独特的工作原理提供了比基于帧的相机更多的潜在优势：高时间分辨率（微秒级）、高动态范围（HDR）（标准相机的60 dB对比为140 dB）、时间冗余抑制以及低功耗（标准相机的1.5 W对比为20 mW）[\[3\]]。它们对于机器人学和计算机视觉中的具有挑战性任务非常有益，例如在高速、极端光照或HDR条件下的自我运动估计[\[4\]]-[\[10\]]以及同时定位与地图构建（SLAM）[\[11\]]-[\[21\]]。
旋转运动估计在视觉和机器人学中是一个基本问题，并且它是更高级运动表述（例如，六个自由度（6-DOF））的基础。尽管经过了数十年的研究，可靠地估计一个纯粹旋转相机的运动仍然是一个挑战，尤其是使用基于帧的相机，因为运动模糊、曝光不足/过度以及图像间大的位移可能会发生并破坏数据关联。相反，事件相机不会受到模糊、低动态范围或数据位之间大位移的影响[\[3\]]，因此，它们可以应对高速和/或HDR旋转运动估计的挑战。挑战在于开发新的算法来处理事件相机产生的非常规、运动依赖的数据，以可靠地建立数据关联，从而实现鲁棒的运动估计。
已有几项工作展示了事件相机在困难场景（高速、HDR）估计旋转运动的能力[\[6\]]、[\[12\]]、[\[15\]]、[\[22\]]。Kim等人[\[12\]]提出了一个3-DOF的同步镶嵌和平移（SMT）方法，由两个并行运行的贝叶斯滤波器组成，以估计相机运动和场景地图。同样并行工作，但使用非线性最小二乘（NLLS）方法，[\[15\]]提出了一种实时全景跟踪和概率映射方法。最近，Kim和Kim[\[22\]]扩展了[\[6\]]中的对比度最大化（CMax）框架，以同时估计角速度和绝对方向。然而，这些方法尚未在统一的标准下进行比较或评估。进行这样的基准测试是具有挑战性的，但也是有用的，以确定促进进一步改进的最佳实践。此外，所有这些方法都是短期的（即，它们估计当前事件集的旋转），对应于SLAM系统的前端[\[23\]]，因此，它们没有考虑长期（即，全局）的优化步骤。这样的步骤，在现代SLAM系统的后端实现，是一个期望的特性，因为它提高了准确性和鲁棒性。
本工作旨在填补这些空白。因此，我们对使用事件相机进行旋转运动估计的问题进行了系统的比较研究，总结了先前的工作，并提出了我们自己对问题的解决方案。我们提出了一种新颖的仅旋转捆绑调整（BA）方法，作为事件相机的后端。与基于帧的BA中使用的重投影或光度误差不同，我们寻求利用最先进的CMax框架[\[25\]]-[\[38\]]来进行BA任务。我们明确地在时间上连续地建模相机轨迹，并使用它将事件变形到全局（全景）地图上。我们将基于事件的BA问题定义为找到实现最清晰（即，运动补偿的）场景地图的旋转相机轨迹。因此，场景地图是优化的内部副产品（见图1）。此外，我们证明了我们的BA在精细化上述基于事件的旋转估计前端产生的轨迹方面的有效性。
我们将所提出的BA（后端）与基于事件的角速度估计器前端[\[6\]]配对，实现了一个名为CMax-SLAM的基于事件的旋转SLAM系统。我们展示了我们的系统能够在具有挑战性的合成和真实世界数据集上可靠工作，包括室内和室外场景，同时优于先前的工作。此外，我们还展示了我们的方法如何处理非常不同的场景，包括将事件相机安装在望远镜上进行恒星跟踪任务产生的数据。
总结来说，我们的贡献如下：1）我们在统一的标准下，理论上比较并实验性地基准测试了几种基于事件的旋转运动估计方法，就准确性和效率而言（见第II节和第IV节）。2）我们提出了第一种基于事件的仅旋转BA方法，以精细化事件相机的连续时间轨迹，同时重建场景边缘的清晰全景地图（见第III-B节）。3）我们提出了一个名为CMax-SLAM的基于事件的仅旋转SLAM系统，首次包括前端和后端（见第III-C节）。4）我们在多种场景中演示了该方法：室内、室外和太空应用，这表明了我们方法的多功能性（见第IV节和第V节）。5）我们强调了在非严格旋转数据上评估仅旋转方法的潜在陷阱，并为现实世界场景中的基于事件的旋转BA提出了一个合理的度量标准（FOM）（见第IV节）。6）我们发布了源代码和具有高空间分辨率的新的独特数据序列。
我们希望我们的工作能够促进对基于事件的自我运动估计领域的更好理解，并帮助承担相关的运动估计任务。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/82be477bee3c4d65b243c76ac2b03344.png#pic_center)

*Fig. 1. Proposed CMax-SLAM system takes as input the asynchronous event stream to estimate the rotational egomotion of the camera while producing a sharp
panoramic map (IWE) as by-product. We can further reconstruct a grayscale map by feeding the events and the estimated trajectory to the mapping module of SMT
[12].*

# 方法

本节首先描述了基于CMax的捆绑调整（BA）的理论公式（见III-A和III-B节）。然后，介绍了CMax-SLAM的流程，包括前端（见III-C1节）和后端（见III-C2节），重点介绍了如何将所提出的BA适应到在线系统中。最后，简要讨论了我们方法的特点（见III-D节）。

## A. CMax旋转运动估计的原理

我们的方法建立在CMax框架[6], [25]提出的事件对齐思想之上。假设光照恒定，当事件相机相对于场景旋转时，图像平面上的边缘模式沿着由相机运动定义的点轨迹移动，产生事件。简单地将事件像素级相加或沿任意点轨迹求和，将得到一个模糊的（或直方图）变形事件图像。相反，沿由相机运动定义的点轨迹对事件求和，将得到一个由事件引起的（运动补偿的）边缘的清晰图像。这一洞见在[6]中得到了利用：在一小包事件的时间跨度内，点轨迹可以通过恒定角速度ω模型参数化，CMax通过寻找其点轨迹与事件最佳对齐的ω（即产生最清晰的IWE）来恢复相机的速度。Shiba等人[38], [44]表明，由该角速度变换定义的点轨迹表现良好，不会遭受“事件塌陷”（即，IWEs不能“过度锐化”）。我们展示了，上述思想可以扩展到更长的时间间隔，因此，需要比恒定ω更复杂的方式来参数化相机运动R(t)。一个连续时间模型R(t)是我们能想到的最通用的模型。虽然CMax目标函数在一般的（6-DOF）相机运动情况下似乎是不适定的[38], [44]，但我们的方法不会遭受事件塌陷。证明见附录A。

## B. 基于事件的捆集调整

BA是帧基视觉SLAM[23], [45]中成熟的技术，通常被表述为最小化重投影误差或光度误差，以精细化相机姿态、场景结构，甚至校准参数。相比之下，异步事件流既不包含可跟踪的运动不变特征，也不包含强度信息，因此，基于帧的BA算法并不直接适用于事件相机。

受[6]和[25]的启发，这些工作通过在短时间IWE中对齐事件来恢复局部运动参数，我们利用CMax来构建事件基础的BA，如下所述。相机轨迹由连续时间模型R(t)描述。每个事件ek根据其对应的旋转R(tk)进行变形，投影并累积到全局IWE中。然后，我们定义旋转BA问题为找到最大化全局地图事件对齐（例如，锐度）的相机轨迹R(t)，通过某种目标函数f来衡量：

$$ \arg\max_{R(t)} f \left[ I(R(t); E) \right] $$

求解上述问题将得到精细化的相机轨迹R*(t)及其相关联的清晰地图M ≡ I*（即，运动补偿的）。由于地图完全由事件和相机轨迹确定，搜索只需要在相机轨迹的空间中进行，而不需要在更大的相机轨迹和地图的空间中搜索。

1) 轨迹参数化：如[17]所述，由于事件相机的高时间分辨率和异步特性，将其轨迹估计或精细化为一组离散时间姿态是次优的。为此，我们使用线性或三次B样条来表示（8）中的连续时间相机轨迹，这些样条由一组时间等间隔的控制姿态在SO(3)中参数化。使用B样条作为连续时间轨迹模型具有以下优点：

a) 它们减少了姿态状态的数量（几个控制姿态与每个事件/时间戳的不同姿态相比）；
b) 它们具有局部支持（每个控制姿态对整体轨迹的影响有限）；
c) 它们统一了，允许我们通过改变一个参数（样条的阶）来选择轨迹的平滑度；
d) 它们使我们能够以原则性的方式融合来自不同传感器的数据（同步和异步），在精确的时间戳查询每个传感器，并；
e) 它们还便于估计传感器之间的时间偏移。

在任何感兴趣的时间点R(t)的姿态是通过一定数量的相邻控制姿态R(ti)的插值获得的。对于线性样条，时间t ∈ [ti, ti+1)的姿态简单地通过两个相邻控制姿态R(ti)和R(ti+1)的李群意义上的线性插值给出，由[46]给出：

$$ R(t) = R(t_i) \exp \left( \frac{t - t_i}{t_{i+1} - t_i} \log \left( R(t_i)^\top R(t_{i+1}) \right) \right) $$

尽管线性样条是连续的，但它们的平滑度有限，因此需要许多控制姿态才能实现足够的平滑度。因此，我们也采用更高阶的样条：三次样条比线性样条更平滑。

在三次B样条的情况下，时间t ∈ [ti, ti+1)的姿态插值需要四个控制姿态，它们出现在{ti−1, ti, ti+1, ti+2}。按照累积三次B样条公式[17], [47]，我们将相机轨迹写为：

$$ R(u(t)) = R_{i-1} \prod_{j=1}^{3} \exp \left( \tilde{B}_j(u(t)) \Omega_{i+j-1} \right) $$

其中u(t) .= (t − ti)/Δt ∈ [0, 1)是归一化的时间表示，Δt .= ti+1 − ti，Ωi .= log(R_i^{-1}R_{i-1})是两个连续控制姿态之间的增量姿态，而˜Bj表示累积B样条基函数的第j个条目（0-based），˜B是三次B样条的矩阵表示。

我们稍微不同地表示线性和三次样条的控制姿态（R(ti)和Ri），因为线性样条通过控制点，而三次样条通常不通过。

尽管连续时间轨迹表示具有优雅性，但其高计算复杂性（尤其是轨迹优化的导数计算）对于将三次B样条应用于轨迹估计是一个挑战。我们通过采用一种最近在[48]中提出的方法来解决这个问题，该方法用于计算具有SO(3)控制姿态的样条轨迹的解析导数。这样，导数计算的复杂性变为样条阶数的线性，这提高了所提出的BA方法的效率。

2) 全景IWE：全局地图I在（8）中是通过在全景图像上旋转和计数对齐的事件来生成的。每个传入的事件ek根据其对应的姿态R(tk)在样条轨迹上旋转：

$$ X'_k = R (tk) X_k $$

其中$X_k = K^{-1} (x_k^\top, 1)^\top$是事件$e_k$在经过校准的相机坐标中的承载方向（3D点），$K$是相机的内在参数矩阵。旋转后的点$\bm X_k^\prime = (X_k^\prime, Y_k^\prime, Z_k^\prime)^\top$然后使用等矩形投影投影到全景上：

$$ p_k = \left( \frac{w}{2} + \frac{w}{2\pi} \arctan \left( \frac{X'_k}{Z'_k} \right), \frac{h}{2} + \frac{h}{\pi} \arcsin \left( \frac{Y'_k}{\sqrt{X'^2_k + Y'^2_k + Z'^2_k}} \right) \right) $$

其中w和h是全景地图的宽度和高度。映射xkW → pk定义了从传感器图像平面到全景地图的事件变形。

最后，变形的事件通过双线性投票在全景地图I上累积，因为它可能没有整数坐标[6], [49]。请注意，事件极性不使用。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/802b84dd5e504524a588a0db3d221a85.png#pic_center)
*Fig. 3. Overview of the proposed rotational event-based SLAM system.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/2ca3cf48b32c42aeb301f4a65fe64741.png#pic_center)
*Fig. 4. Three event slicing strategies: (a) Constant event count, (b) constant
duration, and (c) proposed strategy that selects a fixed number of events around
equispaced timestamps (with output frequency f ). Green and yellow dots repre-
sent processed and skipped events, respectively. Blue arrows are the boundaries
of the event slices. Red dashed lines indicate the selected timestamps for angular
velocity estimation.*

## C. CMax-SLAM系统

所提出的旋转SLAM系统由两部分组成（见图3）。前端接收原始事件作为输入，并使用CMax[6]估计相机的角速度ω。后端通过ω积分获得一组（绝对）旋转，该组旋转用于初始化相机轨迹，然后通过上述BA方法进行精细化。

1) **前端**：我们采用基于CMax的角速度估计器（见第II-B节）作为所提出的SLAM系统的前端，并对其进行修改以与后端配合。该方法假设每个事件包（或“切片”）内角速度是恒定的，因此事件流的适当切片策略至关重要。我们结合了两种常用策略：固定事件计数和固定时间跨度。我们选择一系列等间隔的时间戳（即恒定频率），在这些时间戳处将估计角速度，并在每个时间戳周围切片固定数量的事件。这种事件切片策略类似于文献[18]中的策略；不同之处在于，我们的事件包以所选时间戳为中心，而文献[18]中的包包含所选时间戳之前的事件。如果单个切片的时间跨度太大（在我们的实现中，为 \( \frac{10}{f} \)，其中 \( f \) 是角速度频率），我们假设相机没有移动，因此将角速度设置为零。

2) **后端**：后端BA在滑动窗口结构中工作，其大小固定为时间窗口。当前时间窗口外的控制姿态被固定，而当前时间窗口内的新姿态被初始化和精细化。初始化是通过将前端估计的角速度积分到一组姿态，并使用“提升-解决-重置”方法[46][51]拟合一个新的轨迹段的控制姿态。精细化是通过最大化全局IWE的对比度来完成的，这导致了一个仅依赖于相机轨迹的目标函数：

$$ \arg\max_{\{R_i\}_{i \in R}} \text{Var} \left[ I_L \left( \{R_i\}_{i \in R}, E_{\text{curr}} \right) + \alpha I_G(E_{\text{prev}}) \right] $$

其中 $R$ 是变化控制姿态的索引集，$ E_{\text{curr}} $ 是当前事件，它们被投影以生成局部地图 $ I_L $。全局地图 $ I_G $ 由所有过去的事件 $ E_{\text{prev}} $ 和稳定的姿态构建。$ \alpha $ 是根据 $ I_L $ 和 $ I_G $ 的事件密度自适应计算的权重。

## D. 讨论

CMax-SLAM系统不依赖于显式的引导步骤，并且前端和后端具有一致的目标，即通过最大化对比度来估计运动。前端使用恒定角速度对短时间间隔内的事件进行切片，而后端则使用连续时间旋转模型对更长的时间间隔进行建模。在两种情况下，IWE都作为用于运动估计的亮度图。我们提出的方法在理论上和实验上都展示了其准确性和效率，并且在多种场景下具有广泛的适用性。

# 实验

本节将全面比较所回顾的旋转运动估计方法以及我们提出的BA和CMax-SLAM系统。首先，我们介绍实验设置，包括数据集（见第IV-A节）和评估指标（见第IV-B节）。然后，我们展示合成数据（见第IV-C节）和真实世界数据（见第IV-D节）上的实验结果，并讨论后者的问题。我们还评估了这些方法的运行时间（见第IV-E节），在复杂场景中演示了CMax-SLAM（见第IV-F节），展示了其超分辨率能力（见第IV-G节）。最后，我们进行了灵敏度分析（见第IV-H节）。

## A. 实验设置

1) **数据集**：为了评估旋转估计的准确性和鲁棒性，我们在六个合成序列和六个真实世界序列上进行了实验，这些序列来自标准数据集[22], [53]。所有序列都包含事件、帧（未使用）、惯性测量单元（IMU）数据和地面真实（GT）姿态。

合成序列是使用ESIM模拟器[54]的全景渲染器生成的，输入全景图从互联网下载。生成的序列涵盖室内、室外、白天、夜晚、人造和自然场景。输入全景图的大小从2K（游戏室）、4K（自行车）、6K（城市和街道）到7K（城镇和海湾）分辨率不等。在模拟中，游戏室是一个经典序列，使用DVS128相机模型（128×128像素）生成，持续时间为2.5秒；而其他五个序列则使用DAVIS240C相机模型（240×180像素）生成，持续时间为5秒。

所有六个真实世界数据集都是使用DAVIS240C事件相机记录的，其IMU以1 kHz的速率运行。事件相机数据集（ECD）[53]提供了四个60秒长的旋转运动序列：形状、海报、盒子和动态。前三个序列展示了静态室内场景，随着纹理复杂度的增加，产生的事件数量从大约2000万到2亿不等。最后一个是动态场景（约7000万事件）。相机以增加的速度移动，首先围绕每个轴旋转，然后以3个自由度（3-DOF）自由旋转。运动捕捉（mocap）系统以200 Hz的速率输出GT姿态。对于这四个ECD序列，我们在数据的前30秒进行精度评估，此时平移运动的幅度相对较小。另外两个序列来自[22]：360°室内和快速运动。这两个序列都包含约450万事件，mocap GT姿态以100 Hz给出。在360°室内序列中，相机围绕垂直轴旋转360°，而在快速运动序列中，相机在整个序列中执行随机快速旋转。由于mocap提供的GT有噪声，我们使用以50毫秒时间为跨度的高斯低通滤波器在李群意义上进行了滤波。

2) **硬件配置**：所有实验都在标准笔记本电脑（Intel Core i7-1165G7 CPU @ 2.80GHz）上进行，除了涉及RTPT[15]的实验，由于软件不兼容（运行2017年CUDA代码），这些实验在另一台装有NVIDIA GeForce GTX 970M GPU的笔记本电脑上运行。

3) **下采样**：事件下采样仅在灵敏度研究（见第IV-H节4）和补充材料视频中的实时演示中启用。在所有其他实验中，我们处理所有输入事件。

## B. 评估指标

为了全面描述方法的性能，我们评估了输出轨迹和获得的地图。对于前者，我们计算了相对于GT的估计旋转的绝对和相对误差。对于后者，我们使用每种方法输出的轨迹计算全景图（即IWEs），并扩展了特征基础SLAM中的重投影误差，以评估轨迹和地图的质量。

1) **绝对误差**：绝对旋转误差量化了估计相机姿态的全局一致性。在时间戳 \( t_k \) 处，绝对旋转误差 \( \Delta R_k \) 定义为：

    $$ \Delta R_k = (R'_k)^\top R_k $$

    其中 \( R_k \) 是在 \( t_k \) 处估计的旋转，\( R'_k \) 是相应的GT旋转（通过线性插值得到）。在基准测试中，每种方法以不同的速率输出旋转，因此，我们计算了一系列估计旋转时的绝对误差，并进一步计算了均方根（RMS）以代表精度。

2) **相对误差**：相对旋转误差测量了姿态估计的局部准确性。在时间戳 \( t_k \) 处，相对旋转误差 \( \delta R_k \) 定义为：

    $$ \delta R_k = \left\| (R'_k)^\top R'_{k+\Delta t} - (R_k)^\top R_{k+\Delta t} \right\| $$

    其中 \( \{R_k, R_{k+\Delta t}\} \) 和 \( \{R'_k, R'_{k+\Delta t}\} \) 是基于时间间隔 \( \Delta t \) 选择的估计和GT姿态对。我们设置姿态对的时间跨度为1秒，测量每秒产生的误差，并每秒采样姿态对0.1秒。在实验中，我们使用两个旋转之间的差异角度来测量估计误差：

    $$ \angle B = \arccos\left(\frac{\text{trace}(B) - 1}{2}\right) $$

    其中 $ B $ 是 $ \Delta R_k $ 或 $ \delta R_k $，如文献[56]中采用的。

3) **重投影误差**：对于真实世界数据，由于相机运动不是纯粹的旋转，描述旋转误差存在问题。如第IV-D节1中所述，我们提出通过全景图中的边缘厚度或变形事件占据的区域（即，边缘累积EA）来评估方法的性能。我们使用EA与总图像区域的百分比，因为这更直观。

我们还报告了IWE的梯度幅度（GM）[26]：

$$ \text{GM}(I) = \sqrt{\frac{1}{N} \sum_{\Omega} \|\nabla I(x)\|^2 dx} $$

其中 $ N $ 是像素数量，$ \nabla I $ 是 $ I $ 的空间梯度（例如，使用Sobel算子计算）。

## C. 合成数据上的实验

在这一部分，我们在六个合成序列上比较了前端方法、所提出的BA和CMax-SLAM系统。更多前端细节见附录B。

1) **前端比较**：首先，我们使用合成数据对所有前端进行基准测试。表II的上半部分报告了相应相机轨迹的准确性。

    - **SMT**：总体而言，EKF-SMT在所有前端方法中表现最佳，而PF-SMT仅完成了游戏室序列的跟踪。实验上，EKF-SMT比PF-SMT更稳定，对地图质量也不那么敏感。
    - **RTPT**：由于RTPT的概率操作，可能会在不同试验中输出不同的结果。因此，我们报告了我们获得的最好结果。RTPT在所有合成序列上都失败了，因为它限制了可以跟踪的相机运动范围。这是因为RTPT在操作期间监控跟踪质量，并在质量下降到某个阈值以下时停止地图更新，这通常发生在相机的FOV接近全景图的左侧或右侧边界时。
    - **CMax**：CMax-ω在六个序列中的三个上实现了最佳绝对误差，这证明了事件切片策略（见第III-C1节）的有效性。CMax-GAE在城市、街道和海湾序列上失败了，这些序列有高度纹理化的区域，会触发大量事件。在这些序列上，事件密集地聚集在CMax-GAE的地图上，为其优化器对齐事件带来了障碍。

2) **后端（BA）**：接下来，我们使用所提出的BA方法对不丢失跟踪的前端方法估计的轨迹进行平滑。细化是离线进行的，采用滑动窗口方式（如第III-C2节所述）。

    如表II中间部分所述，所提出的BA能够减少所有不丢失跟踪的前端方法的绝对和相对RMSE。例如，对于自行车序列，EKF-SMT的绝对RMSE从1.382°（前端）降低到0.434°（BA：线性）或0.627°（BA：立方）。CMax-GAE估计的轨迹也通过我们的BA方法得到了显著的改进。轨迹改进在地图质量上也很明显，如Fig. 5所示；细化后的IWE看起来更清晰。

3) **CMax-SLAM**：最后，表II的下半部分报告了所提出的CMax-SLAM系统在相同序列上的准确性。CMax-SLAM在所有合成序列上的表现超过了所有基线方法。此外，从CMax-ω到CMax-SLAM的改进在所有试验中都很明显，这进一步支持了所提出的后端BA的有效性，它以在线方式工作。


  <center> TABLE II ABSOLUTE [◦ ] AND RELATIVE [◦ /S] RMSE ON SYNTHETIC DATASETS

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/1eb4b83d80344cc7884b8e00f76f670f.png#pic_center)


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/55b7a4c4aeb744b5b049f26e66bf74b5.png#pic_center)
*Fig. 5. Effect of BA (offline smoothing). Parts of the panoramic IWEs obtained with the estimated trajectories (before / after BA refinement) and GT. Synthetic data,
as in Table II. Gamma correction γ = 0.75 applied for better visualization.(a) Scenes. (b) EKF-SMT trajectories. (c) Refined EKF-SMT (linear). (d) CMax-GAE
trajectories. (e) Refined CMax-GAE (linear). (f) Groundtruth.*

## D. 在真实世界数据上的实验

本节将比较前端方法、所提出的BA和CMax-SLAM在真实世界数据上的性能。

1) **评估问题**：真实世界评估的主要难点在于找到与纯粹旋转运动假设兼容的真实数据。真实世界序列通常包含平移运动，而所有方法都施加了旋转运动约束。这导致在非严格3-DOF数据上评估3-DOF方法时出现问题。

由于无法从输入事件中分离出旋转和平移部分，所有方法都尝试仅使用旋转自由度来解释额外的自由度。如果平移运动不可忽视，将这些旋转与由6-DOF运动捕捉系统提供的地面真实（GT）的旋转部分进行比较可能会产生误导性的结果（增加均方根误差RMSE）并导致与视觉结果不一致（例如，地图中出现双重和模糊的边缘）。因此，需要一个更合理的度量标准（FOM）来表征旋转估计方法的性能。

在传统的BA中，重投影误差是衡量未知数（场景结构/地图和相机运动）与视觉数据拟合好坏的指标。在事件基础的旋转运动估计中，重投影误差可以通过观察全景图（IWE）中的边缘模糊或多重边缘来间接评估。因此，我们使用边缘累积（EA）作为重投影误差的代理，它反映了全景图中变形事件的分布。

2) **前端比较**：表III的上半部分报告了所有前端在真实世界序列上估计的轨迹的RMSE。此外，图6展示了ECD序列[53]上所有前端产生的轨迹与GT的比较，而相应的误差统计显示在图7中。

    - **SMT**：在所有ECD序列中，PF-SMT仅能完成360°室内序列的完整跟踪。EKF-SMT在快速运动序列上报告了最高的准确性，但在360°室内序列上与PF-SMT表现相似。然而，SMT在所有ECD序列上最终都失败了。SMT在开始时能准确跟踪（得益于稳定的IMU初始化），但随后会在某个点突然失去跟踪，而不是逐渐累积漂移。跟踪失败通常发生在相机突然改变旋转方向时（例如，回头）。我们怀疑这是由于跟踪和映射线程之间的误差传播所致。

    - **RTPT**：RTPT能够在相机的FOV围绕全景图中心移动的序列上表现良好[53]。然而，当相机探索更大的区域时，RTPT报告的误差急剧增加，甚至失去跟踪。这再次证明了其在可跟踪相机运动范围上的局限性。

    - **CMax**：CMax-ω前端在所有序列上都计算出了准确的角速度，表现良好。在真实世界数据上，它显示出鲁棒性，并且不会产生误差的急剧增加。CMax-GAE通过联合估计角速度和绝对旋转，提供了更好的长期跟踪一致性，如其在大多数真实世界序列上的竞争性能所示。由于高纹理区域会触发大量事件，CMax-GAE在盒子序列上失败了。

3) **后端（BA）**：接下来，我们的离线BA方法在真实世界数据上进行了测试。BA的改进在相机轨迹上最为显著，这些轨迹与GT有相当大的漂移，如IMU航位推算。图8显示了ECD序列的估计相机轨迹在细化前后的情况。细化后的轨迹非常接近GT，并且绝对误差分布比初始的更小、更集中（见图9和表III的中间部分）。相对误差没有显著变化，这是合理的，因为它们大约测量角速度误差，而IMU提供了准确的角速度数据。

图10提供了与旋转相关的轨迹在细化前后生成的全景IWE的定性比较。地图在细化后变得相当清晰。表IV报告了相应的重投影误差（EA）也在BA细化后减少。为了比较，图10的最后一列显示了使用GT旋转获得的地图；它们不是很清晰（因为相机运动不是纯粹的旋转），这也反映在更高的重投影误差（EA）上，与细化后的旋转相比。

4) **CMax-SLAM**：最后，我们在相同的序列上测试了所提出的CMax-SLAM系统。估计的轨迹和误差在图Figs. 12和13中绘制，RMSE数字在表III的底部给出，代理重投影误差（EA）在表IV中给出。CMax-SLAM估计的轨迹非常接近GT（见图12）。小提琴图在图13中显示，在大多数情况下，绝对误差减少，其分布比CMax-ω前端更集中。在形状和动态序列上，效果最为显著，其中平移运动的数量比海报和盒子少。相对误差几乎没有变化，因此，没有破坏前端估计的准确角速度。在代理重投影误差方面，CMax-SLAM在所有ECD序列上都实现了最小值（见表IV）。
<p align="center">
  <img src="./table/table3.png" alt="" title="" style="display: block; margin: 0 auto;" />
  TABLE III
ABSOLUTE [◦ ] AND RELATIVE [◦ /S] RMSE ON REAL-WORLD DATASETS
</p>

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4673f5f14fc747a7966666041694f82b.png#pic_center)

*Fig. 6. Front-ends (before BA). Comparison of camera trajectories of all front-end methods involved in the benchmark. EKF-SMT and PF-SMT do not show up
because they fail all sequences from ECD [53] dataset. (a) shapes. (b) poster. (c) boxes. (d) dynamic.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/d62897097b9b4e96b451c52ce1c293cf.png#pic_center)
*Fig. 7. Front-ends (before BA). Absolute and relative errors of the visual front-end methods. SMT is not included since it does not work on most sequences. (a)
shapes. (b) poster. (c) boxes. (d) dynamic.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/30bd49c8346a4e848e28dbfef654e1be.png#pic_center)
*Fig. 8.
Camera trajectories: IMU dead reckoning and its refined trajectories (linear and cubic) using BA. (a) shapes. (b) poster. (c) boxes. (d) dynamic.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4b88a85d64a946d0bbb6d99c6bdd851a.png#pic_center)
*Fig. 9. Refinement of IMU dead reckoning using BA (offline smoothing). Absolute and relative errors of the input and refined trajectories. “l” and “c” indicate the
BA algorithm with linear and cubic splines, respectively. (a) shapes. (b) poster. (c) boxes. (d) dynamic.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/213c1b4de2b9447c93ab594fdcab7574.png#pic_center)
*Fig. 10. Effect of BA (offline smoothing). Central part of the panoramic IWEs generated using the estimated trajectories (before/after BA refinement) and GT. Gamma correction: γ = 0.75. Data from ECD [53], using events in [1,11]s. (a) Scene. (b) IMU dead reckoning. (c) Refined trajectories (linear splines). (d) Refined trajectories (cubic splines). (e) Groundtruth (only rotation).*

<p align="center">
  <img src="./table/table4.png" alt="" title="" style="display: block; margin: 0 auto;" />
  TABLE IV
PROXY REPROJECTION ERROR GIVEN BY THE EA OF THE PANORAMIC IWE
</p>

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/9026c2818d95498687121970cec4ce15.png#pic_center)
*Fig. 11. Effect of BA (offline smoothing). Central part of the panoramic grayscale maps generated using the estimated trajectories (before/after BA refinement)
and GT. The grayscale map size is 1024 × 512 px. Same events as Fig. 10. (a) CMax-GAE trajectories. (b) Refined CMax-GAE trajectories, linear splines. (c)
RTPT trajectories. (d) Refined RTPT trajectories, linear splines. (e) Groundtruth (only rotation).*

## E. 运行时间评估

1) **前端比较**：为了比较它们的效率，我们在ECD数据集的选定片段上运行了所有前端，并测量了它们的处理时间（见表V）。所有前端都处理所有事件，除了CMax-GAE，它采样了一个四分之一的事件（即，只处理了25%的事件，如其原始实现）。EKF-SMT在高纹理场景（海报和盒子）中比其他方法更快，而CMax-GAE在低纹理场景（形状和动态）中报告了最短的运行时间。CMax-ω在处理所有事件的同时具有竞争力，并以100 Hz的频率产生姿态（与CMax-GAE的40 Hz相比）。总的来说，除了在形状序列上，其纹理非常简单，否则没有前端方法显示出实时性能。

2) **CMax-SLAM**：表VI报告了CMax-SLAM在几个序列上的计算成本，这些序列在两种不同的事件相机分辨率下：240×180像素（ECD数据集[53]中的DAVIS240C）和346×260像素（DAVIS346）。它显示，相机分辨率越高，处理时间越长。此外，三次样条轨迹表示的计算成本合理地高于线性的。目前，CMax-SLAM系统的实现没有针对实时性能进行优化。通过处理较少的事件和减小全景图的大小，它有潜力实现近实时性能。

## F. CMax-SLAM在野外的实验

为了证明CMax-SLAM能够在复杂的自然场景中可靠工作（即，具有任意的光照变化），我们在没有GT的野外序列上测试了它。

我们为旋转运动研究创建了一个新的真实世界事件相机数据集（见表VII），其中包含了使用DVXplorer从iniVation AG（640×480像素）记录的十个序列。所有序列都有事件和IMU数据（运行频率约为800 Hz）。对于一些序列，我们将相机放置在电机化支架（Suptig RSX-350）上，以产生围绕Y轴的近似均匀旋转运动。尽管使用了电机化支架，但由于旋转中心仍然偏离相机的光学中心，相机无法执行纯旋转运动。对于其余序列（交叉路口、桥和河流），相机是手持的，这可能会引入更多的不规则残余平移。

与[22], [58]中的mocap房间中的六个室内序列相比，这些户外序列包含了更困难的光照条件（例如，河流和窗户中的反射）、动态对象（例如，移动的行人、自行车、汽车、树叶和河上的水）以及直接阳光照射导致的镜头眩光，这使得该数据集更具挑战性。

我们在上述序列上运行CMax-SLAM并生成全景IWE。图14中的结果表明，CMax-SLAM为上述具有挑战性的场景恢复了精确的全局、清晰的IWE。补充材料视频中展示了额外场景的结果。

## G. 超分辨率

1) **CMax-SLAM的超分辨率**：后端中全景地图的分辨率在一定程度上独立于事件相机的分辨率。图15显示了在不同地图分辨率下运行CMax-SLAM的结果，从512×256像素到8192×4096像素。地图越大，内存需求就越大，IWE中使用的双线性投票就越慢（由于内存访问，尽管投票复杂性与事件数量成线性关系），因此优化也越慢。如图16(a)所示，每次将地图大小加倍，处理一个事件的CMax-SLAM后端（BA）的计算成本大约呈指数级增加。一个非常低分辨率的地图会将许多事件变形到少数几个像素中。一个非常大的分辨率地图可能会包含空像素（没有变形的事件），这些空像素可以通过平滑地图来填充。在两种极端情况下，自我运动算法可能会失败。在中间情况下[见图15，列(b)-(d)]，CMax-SLAM能够良好工作，产生清晰的地图，因为连续时间的相机轨迹和高时间分辨率的事件允许我们以几乎连续的方式变形事件，从而将高时间分辨率转换为高空间分辨率。合理的地图大小选择是使个别地图像素覆盖与事件相机相同的场景区域。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/7592b2a8996f48ab837479cbb6bac730.png#pic_center)

*Fig. 12. CMax-SLAM (online). Trajectory comparison of CMax front-end and CMax-SLAM (linear and cubic). (a) shapes. (b) poster. (c) boxes. (d) dynamic.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/30134f117f254b9fa78355831db94528.png#pic_center)


*Fig. 13. CMax-SLAM (online). Absolute and relative errors of the compared CMax-SLAM. “l” and “c” indicate the CMax-SLAM with linear and cubic spline
trajectories, respectively. (a) shapes. (b) poster. (c) boxes. (d) dynamic.*

<p align="center">
  <img src="./table/table5.png" alt="" title="" style="display: block; margin: 0 auto;" />
  TABLE V
RUNTIME EVALUATION OF COMPARED FRONT-END METHODS [S]
</p>

<p align="center">
  <img src="./table/table6.png" alt="" title="" style="display: block; margin: 0 auto;" />
  TABLE VI
CMAX-SLAM RUNTIME EVALUATION [μS/EVENT PROCESSED], FOR A MAP
OF SIZE 1024 × 512 PX
</p>

2) **灰度图重建的超分辨率**：在另一个实验中，我们在1024×512像素的分辨率下运行CMax-SLAM，并使用估计的轨迹和事件作为输入，提供给SMT的映射模块。图17比较了在不同分辨率下重建的全景灰度图，从512×256像素到8192×4096像素。因此，连续的轨迹和事件的准确计时使我们能够达到超分辨率。随着分辨率的增加，场景的更多细节被恢复（例如，图17第二行的棋盘格）。从图17的每一列移动到下一列，像素数量$N_p$增加四倍，双线性投票由于内存访问而变得更慢，通过快速傅里叶变换（FFT）的泊松重建的成本也大约增加了四倍。每当地图大小加倍，用于SMT映射模块中的EKF的运行时间大致线性增加，但FFT的泊松重建成本却显著增加。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/16ee90f2160c4edf81186c63880aaf9e.png#pic_center)

*Fig. 14. Experiments in the wild. Panoramic IWEs produced by CMax-SLAM at 4096 × 2048 px resolution, on the data from a 640 × 480 px camera. The map sharpness is a proxy for the motion estimation quality. Gamma correction: γ = 0.75.*

<p align="center">
  <img src="./table/table7.png" alt="" title="" style="display: block; margin: 0 auto;" />
  TABLE VII
SELF-RECORDED DATASET WITH HIGH RESOLUTION EVENT CAMERA;
DESCRIPTION OF SEQUENCES
</p>

## H. 灵敏度分析

最后，我们评估了我们方法对几个参数的灵敏度（见图19）。

1) **控制姿态频率**：在这组实验中，我们将时间窗口大小设置为0.2秒，全景IWE的分辨率设置为1024×512像素。结果在图19(a)中展示。对于自行车（合成数据），CMax-SLAM的准确性几乎不随控制姿态频率从10 Hz变化到40 Hz而变化。对于动态（真实世界数据），当控制姿态频率从20 Hz降低到10 Hz时，误差明显增加。看来控制姿态频率一旦达到某个值（例如，对于动态的20 Hz），就不会是影响准确性的主要因素。

2) **时间（滑动）窗口大小**：在这组实验中，我们将控制姿态频率设置为20 Hz，全景IWE的分辨率设置为1024×512像素。我们用于滑动时间窗口的步长是其大小的一半。总体而言，CMax-SLAM的准确性随着时间窗口大小的增加而在自行车和动态上都略有下降。自行车序列由于仅长5秒，因此没有测试2秒的窗口大小。

3) **全景IWE分辨率**：在这组实验中，我们将控制姿态频率设置为20 Hz，时间窗口大小设置为0.2秒。如图19(c)所示，随着地图大小的增加，误差减少。然而，如第IV-G节所述，CMax-SLAM工作的正常地图大小应该有下限和上限。此外，随着地图大小的增加，计算成本迅速增加。

4) **事件采样率**：在这组实验中，我们将控制姿态频率设置为20 Hz，时间窗口大小设置为0.2秒，全景地图分辨率设置为1024×512像素。事件在输入到前端和后端之前被系统地下采样。具体来说，事件采样率为5意味着前端和后端处理一个事件中的一个（即20%的事件）。如图19(d)所示，动态对事件采样率直到10都不敏感。相比之下，自行车的准确性随着事件采样率的增加而逐渐下降。

此外，图19(e)展示了事件采样率如何影响CMax-SLAM的处理时间：如预期，随着处理的事件减少，运行时间减少，但不是线性的（处理一半的事件并不减少一半的运行时间）。在所有情况下，三次B样条总是比线性的更昂贵。通过图19(d)和(e)，用户可以为他们自己的应用设置权衡。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ce1c7076c17f45de94541291b1f6043d.png#pic_center)
*Fig. 15. CMax-SLAM at several resolutions. Results of running CMax-SLAM at different map resolutions (columns), and using the estimated trajectories to
generate panoramic IWEs at the corresponding resolutions. Gamma correction: γ = 0.75. Data from boxes (same events as Fig. 10). (a) 512 × 256 px. (b) 1024 ×
512 px. (c) 2048 × 1024 px. (d) 4096 × 2048 px. (e) 8192 × 4096 px.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/5b0bb249ddee4034be509532d74dff73.png#pic_center)
*Fig. 16. Results of running CMax-SLAM at super resolution (Fig. 15). (a)
Runtime evaluation of CMax-SLAM back-end at different resolutions. (b) Map
quality evaluation of running CMax-SLAM at different resolutions. The lower
the NIQE and PIQE scores, the better.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ad0e1a552cfa4b8fb5383b71e51a2fbb.png#pic_center)

*Fig. 17. Grayscale map reconstruction at super-resolution. Results of running CMax-SLAM at 1024 × 512 pixel resolution to estimate the camera trajectory
and feed it to the mapping module of SMT to obtain panoramic grayscale maps at different resolutions (columns). The location of the zoomed-in region is indicated
with red rectangles. Same events as Fig. 10. (a) 512 × 256 px. (b) 1024 × 512 px. (c) 2048 × 1024 px. (d) 4096 × 2048 px. (e) 8192 × 4096 px.*

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/643d268455c04f818b4972027ca988bc.png#pic_center)

*Fig. 18. Results of grayscale map reconstruction at super resolution (Fig. 17).
(a) Runtime evaluation of Poisson reconstruction at different resolutions. (b)
Map quality evaluation of grayscale map reconstruction at different resolutions.*

## 空间应用

事件相机的低延迟、HDR和低功耗特性使它们在空间应用中具有吸引力。例如，作者在[64], [65]中将事件相机应用于恒星跟踪问题，该问题由跟踪恒星来估计旋转相机的自我运动。数据可以通过地球上或卫星上的事件相机获取。

让我们将所提出的方法应用于解决恒星跟踪问题，同时重建全景星图。实验表明，我们的方法在准确性和鲁棒性方面优于先前的工作。我们的方法不将事件转换为帧，也不提取对应关系，因为它基于CMax[25]，这隐式地处理了数据关联（通过变形事件对同一像素的投票）。我们的BA也不同于先前在恒星跟踪中使用的BA，后者是基于特征的（在2-D-3-D点对应关系之间）在将事件转换为帧之后[64]，或者拟合线段并提取它们的端点[65]。因此，我们的方法比先前的工作更适应事件数据的特性。

### A. 实验

1) **数据集**：我们在星数据集[64]上测试了所提出的方法。它包含十一个子序列，每个序列长45秒，使用DAVIS240C事件相机（240×180像素）记录，同时观察以恒定角速度4°/s旋转的星场，显示在屏幕上。与[64], [65]一样，实验在序列1至6上进行，这些序列包含1.4至6.2 M事件。由于数据提供的格式（文本文件和浮点数未畸变事件，使用组合单应性-校准矩阵）与我们实现的直接使用原始事件和已知相机校准矩阵的格式不兼容，我们离线运行了前端和BA。

2) **指标**：先前的工作[64], [65]通过提供额外的一组绝对姿态（从星体识别系统获得的锚点）来进行旋转平均[66]，以便于轨迹精细化。由于数据集中没有提供这样的数据，我们只能比较相对旋转估计的性能。为了直接比较，我们采用了[64]中的基准：在400毫秒的时间间隔内测量相对旋转之间的角距离。我们还使用了[64]中的指标：均方根误差（RMSE）和标准差。

3) **结果**：表VIII将我们的方法与现有技术进行了比较。我们的方法产生了准确的结果（<1° RMSE）：在序列1、2、4中优于现有技术，在其余序列中与之相当。序列3是众所周知的具有挑战性的[65]。此外，我们的方法是最一致的，如其较小的标准差所示，与其他方法相比。

图20显示了我们方法的一个序列的输出。由于序列中没有环路，没有额外输入（锚点绝对姿态）的情况下，图20(b)中的旋转漂移是不可避免的。然而，估计仍然是好的：角速度误差很小[见图20(a)]，由前端生成的局部IWE（使用事件极性）在图20(c)和(d)中显示得很清晰，表明该方法已成功估计了引起事件数据的运动。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/cafab7700f7b48ebb7f179c4cbf142a5.png#pic_center)

*Fig. 19. Sensitivity Analysis. From left to right, effect of varying: The frequency of the control poses, the size of the time window, the resolution of the IWE map,
and the event sampling rate (on absolute rotation errors and runtime).*

<p align="center">
  <img src="./table/table8.png" alt="" title="" style="display: block; margin: 0 auto;" />
  TABLE VIII
SPACE APPLICATION ERRORS OF RELATIVE ROTATIONS (RMSE AND STANDARD DEVIATION σ) [◦ ]
</p>

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/5b3c21748e2143529f920ed728917566.png#pic_center)
*Fig. 20. Space Application. Results on sequence 5 of [64]: (a) Estimated angular velocity. (b) Absolute rotation. (c) Local IWE before/after CMax (generated by
the front-end with the event polarity used), corresponding to the region marked by the red rectangle in (d). (d) A zoomed-in part of motion-compensated event-based
star map (generated by the back-end with the event polarity not used). In (a) and (b) blue solid lines are estimated whereas red dashed lines are the GT.*

## 局限性

事件相机需要场景中存在对比度。在无纹理区域或边缘较少的情况下，很难进行跟踪。在数据缺失的情况下，可以通过调整相机参数，降低对比度阈值$C$来生成更多事件，希望能感应到小边缘。所有比较的方法都假设亮度恒定。由闪烁的灯光或热像素引起的事件不遵循此假设，如果它们支配了由移动边缘产生的事件，可能会引起问题。在实践中，我们的方法对场景中的少数闪烁灯光（例如，来自mocap系统的残余光）表现出一定的鲁棒性。

虽然我们的系统包括了一个用于精细化的后端，但它缺乏具有环路闭合能力的显式模块（所有测试的方法都缺乏它），因此不可避免地会累积漂移。这在场景在完整的360°旋转后被重新访问的序列中最为明显。据作者所知，事件基础旋转-自我运动估计的文献中还没有方法能够检测环路闭合（我们甚至敢将这一点扩展到事件基础的6-DOF自我运动估计文献中）。循环闭合检测在事件视觉领域仍处于起步阶段，因此作为未来工作留下。

具有640×480像素及以上空间分辨率的事件相机可能会产生大量事件（例如，1Gev/s[2]），如果使用所有事件，这将需要大量的内存和计算能力。更高效的处理器，理想情况下是像神经形态处理器这样的大规模并行处理器，可以解决这个问题。

## 结论

我们提出了第一个基于事件的旋转-捆集调整（BA）和第一个包含前端和后端的SLAM系统。两者都基于事件对齐（CMax），后端具有连续时间相机轨迹模型。我们展示了两种轨迹模型（线性和三次B样条），它们的准确性相似，但复杂性不同（三次样条要求更高）。其他连续时间模型，如高斯过程，也可能使用。据作者所知，以前关于同一任务的工作没有考虑过去事件进行轨迹精细化，而没有将它们转换为帧。

虽然CMax问题不是以非线性最小二乘（NLLS）问题的形式表述的（因此不适用强大的二阶算法，如高斯-牛顿法），但我们有效地使用了一阶非线性共轭梯度法。我们解析地计算导数，并仅在轨迹空间中搜索，因为地图自然由轨迹、事件和投影模型确定。这使得潜在的实时操作成为可能，具体取决于事件速率和平台的处理能力。

这项工作提供了旋转估计方法与事件相机的最全面基准测试，评估了先前的工作，并重新实现了作者认为其代码不可用的方法。实验表明，我们的方法在合成数据上优于所有基线方法。在真实世界数据上，我们提出了一个更合理的度量标准（FOM）来评估基于事件的旋转估计；结果表明，前端与现有技术相当；后端与前端紧密集成，能够通过小的代理重投影误差和清晰的地图来精细化轨迹。

最后，由于我们方法的多功能性（实验表明它在室内、室外、自然场景和空间数据中均有效），它可以在卫星和漫游车上使用。它还可以用于天空映射（使用宽视场镜头，因为对于窄视场镜头，简单的线性近似足以用于事件变形）。因此，我们的工作直接影响了基于事件的HDR天空映射、态势感知（SDA）和空间机器人学。我们希望我们的工作能够激发出推进自我运动估计领域的想法（不仅在3-DOF，而且在更高的DOFs）。
