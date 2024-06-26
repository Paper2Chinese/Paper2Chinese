
# [Unified 3D and 4D Panoptic Segmentation via Dynamic Shifting Networks](https://ieeexplore.ieee.org/document/10380457/)
## 题目：通过动态移位网络实现统一的3D和4D全景分割
**作者：Fangzhou Hong; Lingdong Kong; Hui Zhou; Xinge Zhu; Hongsheng Li; Ziwei Liu**  
**源码：https://github.com/hongfz16/DS-Net**  
****
# 摘要

随着自动驾驶技术的快速发展，为其感知系统配备更全面的三维感知能力变得至关重要。然而，像三维检测或点云语义分割这样被广泛探索的任务，主要集中在解析物体或场景。在这项工作中，我们提出了解决基于激光雷达的全景分割的挑战性任务，该任务旨在以统一的方式解析物体和场景。特别是，我们提出了动态偏移网络(DS-Net)，它作为点云领域有效的全景分割框架。DS-Net 具有针对复杂激光雷达点云分布的动态偏移模块。我们展示了一个高效的可学习聚类模块，动态偏移，它适应不同实例的核心函数。为了进一步探索时间信息，我们将单扫描处理框架扩展到其时间版本，4D-DS-Net，用于四维全景分割任务，其中跨多帧的同一实例应给出相同的ID预测。与简单地将跟踪模块附加到DS-Net不同，我们提出以更统一的方式解决四维全景分割问题。具体来说，4D-DS-Net 首先通过对齐连续的激光雷达扫描构建四维数据体，在此基础上执行时间统一的实例聚类以获得最终结果。在两个大规模自动驾驶激光雷达数据集SemanticKITTI和Panoptic nuScenes上进行了广泛的实验，以证明所提出解决方案的有效性和优越性能。

# 关键词

- 四维全景分割
- 激光雷达全景分割
- 点云语义和实例分割

# I. 引言

自动驾驶作为计算机视觉最有前景的应用之一，在最近几年中取得了迅猛的进步。感知系统是自动驾驶中最重要的模块之一，也吸引了之前研究工作中的广泛研究。不可否认，传统的3D目标检测[1]、[2]、[3]和语义分割[4]、[5]、[6]、[7]、[8]任务已经发展出成熟的解决方案，这些方案支持现实世界中的自动驾驶原型。然而，这些任务与对于具有挑战性的自动驾驶场景至关重要的整体感知目标之间，仍然存在相当大的差距。在这项工作中，我们提出通过探索基于激光雷达的3D和4D全景分割任务来缩小这一差距，这需要在时空域中进行密集的点级预测。

全景分割对于图像[9]和视频[10]已被提出，作为统一语义和实例分割的新视觉任务。Behley等人[11]将此任务扩展到激光雷达点云，并提出了基于激光雷达的全景分割任务。其时间对应物也作为4D全景分割[12]被引入，用于在时间维度上提供更一致的感知。如图1(a)所示，基于激光雷达的全景分割需要对背景（物体）类别（例如道路、建筑和植被）进行点级语义标签的预测，而实例分割则需要对前景（事物）类别（例如汽车、人和骑自行车的人）进行。4D全景分割进一步要求跨不同帧的同一实例被赋予相同的ID。这两项任务都带来了空间和时间视角的挑战，以下将进行讨论。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/4d2d476307e34bec8b314fdac00b5cb3.png" width="70%" /> </div>


从空间角度来看，激光雷达扫描的复杂点分布使得执行可靠的全景分割变得困难。大多数现有的点云实例分割方法[13]、[14]主要设计用于密集且均匀的室内点云。因此，通过中心回归和启发式聚类算法可以获得不错的分割结果。然而，由于激光雷达点云的非均匀密度和实例大小的变化，中心回归未能为聚类提供理想点分布。回归的中心通常形成密度和大小不一的嘈杂条带状分布。正如第III-B节所分析的，先前工作中广泛使用的几种启发式聚类算法无法为激光雷达点云回归中心提供令人满意的聚类结果。为了应对上述技术挑战，我们提出了动态偏移网络（DS-Net），它专门设计用于有效的激光雷达点云全景分割。

首先，我们采用了一个强大的主干网络设计，并为这项新任务提供了一个坚实的基线。受到[15]的启发，我们使用了圆柱卷积来高效地提取每个激光雷达帧的网格级特征，并且这些特征进一步由语义和实例分支共享。

其次，我们在实例分支中提出对每个点的实例中心进行回归。然后，我们设计了一个新颖的动态偏移模块，用于对回归中心进行聚类，这些中心通常具有复杂的分布。如图1(b)所示，所提出的动态偏移模块将回归中心  $p_i$  移动到聚类中心  $x_i$  。聚类中心  $x_i$  是通过对几个邻近点  $c_{ij}$  通过核函数  $k_j$  计算并加权得到的。该模块的特殊设计使得偏移操作能够动态适应不同实例的密度或大小，因此在激光雷达点云上表现出色。进一步的分析还表明，动态偏移模块是稳健的，并且对参数设置不敏感。

从时间角度来看，跨帧有效关联实例并非易事，特别是当它们在移动并且部分被遮挡时[16]。稀疏的激光雷达点云对其外观提供的线索很少，这使得它们之间很难区分。因此，基于多次单次扫描分割结果进行跟踪是次优的，正如第IV-D节所证明的。相反，我们提出以更统一的方式来处理4D全景分割，以便充分利用连续帧的信息并隐式关联。

类似于4D-PLS[12]，我们采用SLAM系统估计的姿态来对齐和重叠连续的激光雷达扫描，形成4D数据体。然后设计了时间统一的实例聚类，以帧不可知的方式执行实例分割，其结果进一步与语义分割融合，形成最终的4D全景分割结果。这种在4D数据体上的统一分割避免了复杂后跟踪模块的需要。与此同时，信息提取过程完全具有时空意识，使其比“分割然后跟踪”的方法更有效。

在SemanticKITTI[17]和Panoptic nuScenes[18][19]这两个大规模自动驾驶数据集上的广泛实验，证明了我们提出的DS-Net和4D-DS-Net在基于激光雷达的3D和4D全景分割上的有效性。为了进一步展示这些任务的挑战，我们还展示了几个通过结合最先进的语义分割、检测和跟踪方法得出的强基线结果。DS-Net和4D-DS-Net在所有测试基准上都表现出了竞争力。还进行了更多的分析，以提供对动态偏移模块更深入的见解。

主要贡献总结如下：
- 提出的DS-Net有效处理了激光雷达点云的复杂分布，并在大规模的SemanticKITTI和Panoptic nuScenes数据集上取得了竞争性能。
- 我们通过提出4D-DS-Net展示了一个简单但有效的4D全景分割解决方案。与DS-Net一起，我们制定了一个统一的3D和4D全景分割框架。
- 进行了广泛的实验来证明有效性。进一步进行了统计分析，以提供有价值的观察。


# III. 我们的方法

本节我们分为两部分介绍，第一部分是单次扫描版本的DS-Net，基于此进一步引入其4D对应版本4D-DS-Net。对于DS-Net部分，如图2所示，我们首先介绍了一个强大的骨干设计来建立一个简单的基线（第三节A），基于此进一步提出了两个模块。提出了新颖的动态偏移模块来解决非均匀激光雷达点云分布的挑战（第三节B）。高效的多数投票策略结合了语义和实例预测，并产生了全景分割结果（第三节C）。第二部分，我们介绍了一个简单但有效的扩展，即4D-DS-Net，用于4D全景激光雷达分割任务（第三节D）。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/064765a57c4c49c789a73955b9fa6749.png" width="70%" /> </div>


## A. 强大的骨干设计
为了获得全景分割结果，很自然地分别解决两个子任务，即语义分割和实例分割，并组合结果。如图2的上半部分所示，强大的骨干由三部分组成：圆柱卷积、语义分支和实例分支。通过圆柱卷积从原始激光雷达点云中提取高质量的网格级特征，然后由语义和实例分支共享。

**圆柱卷积**：考虑到任务所呈现的困难，我们发现圆柱卷积[15]最符合高效率、高性能和完全挖掘3D位置关系的严格要求。圆柱体素划分可以比正常的笛卡尔体素划分产生更均匀的点分布，因此可以提高特征提取效率和性能。圆柱体素表示结合稀疏卷积可以自然保留并充分探索3D位置关系。具体来说，我们使用稀疏卷积构建了一个在圆柱体素上操作的U-Net[92]。输入的激光雷达点云 $P \in \mathbb{R}^{N \times 4}$ 由N个点组成，每个点 $p_i$ 具有四个属性，分别代表其XYZ坐标 $(x_i, y_i, z_i)$ 和相应反射光束的强度 $r_i$ 。骨干网络的输出是体素特征 $F_v \in \mathbb{R}^{H \times W \times L \times D}$ ，其中D表示特征的维度。H、W、L分别是体素分辨率，在实践中取值为480、360、32。

**语义分支**：通过对骨干网络的体素特征 $F_v$ 应用卷积，预测每个体素的语义逻辑 $L_s \in \mathbb{R}^{H \times W \times L \times C}$ ，其中C是所有类别的数量，H、W、L代表体素维度，然后通过softmax操作计算每个体素的预测语义标签。通过将体素标签复制到体素内的点上，获得点级的语义预测。考虑到自动驾驶场景中的类别不平衡，我们选择加权交叉熵损失和Lovasz损失[93]作为语义分割分支的损失函数。

**实例分支**：实例分支利用中心回归为进一步聚类做准备。中心回归模块使用MLP适应圆柱卷积特征，并使事物点回归到它们的实例中心，通过预测偏移向量 $O \in \mathbb{R}^{M \times 3}$ 指向从点 $P \in \mathbb{R}^{M \times 3}$ 到实例中心 $C_{gt} \in \mathbb{R}^{M \times 3}$ ，其中M代表语义分割预测的事物点的数量。实例分支的损失函数可以表述为：

$$
L_{ins} = \frac{1}{M} \sum_{i=0}^{M-1} \| O[i] - (C_{gt}[i] - P[i]) \|_1,
$$

其中M是事物点的数量。回归的中心 $O + P$ 进一步聚类以获得不同的实例，然后为它们分配实例ID。这可以通过启发式聚类算法或接下来介绍和分析的提出的动态偏移模块来实现。

## B. 动态偏移
点聚类重访：与从重建网格中采样的室内点云不同，激光雷达点云的分布不适合室内实例分割方法中使用的正常聚类解决方案。激光雷达点云的不同实例大小、稀疏性和不完整性使得中心回归模块难以预测精确的中心位置，并将导致嘈杂的长“条状”分布，如图1（b）所示，而不是围绕中心的理想球形聚类。此外，如图3（a）所示，远离激光雷达传感器的聚类形成的群集密度远低于附近的群集，因为点云稀疏性取决于与传感器的距离。面对回归中心的非均匀分布，启发式聚类算法难以产生满意的结果。下面分析了在以前的基于底部的室内点云实例分割方法中使用的四种主要启发式聚类算法。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/0f581ea671fe48699b1c879eed94b115.png" width="70%" /> </div>


**广度优先搜索（BFS）**：BFS简单且对室内点云来说足够好，如[14]所证明的，但不适合激光雷达点云。如上所述，不同群集之间的密度差异意味着固定半径不能适当地适应不同的群集。小半径倾向于过度分割远距离实例，而大半径倾向于欠分割近距离实例。

**DBSCAN[94]和HDBSCAN[95]**：作为基于密度的聚类算法，不出所料，这两种算法在激光雷达点云上的表现也很差，即使它们被证明对室内点云聚类[13]、[96]是有效的。DBSCAN的核心操作与BFS相同。HDBSCAN直观地假设低密度的点更可能是噪声点，这与激光雷达点不同。

**均值漂移[97]**：均值漂移的优势在于，它对密度变化不敏感，对噪声点鲁棒，这使得它比基于密度的算法更适合，如[98]用于聚类室内点云。然而，核函数的带宽对聚类结果有很大的影响，如图3（b）所示。固定带宽无法同时处理大型和小型实例的情况，这使得均值漂移不是此任务的理想选择。

**动态偏移**：如上所述，通过迭代应用核函数来稳健估计回归中心的聚类中心，如均值漂移中所示。然而，核函数的固定带宽未能适应不同的实例大小。因此，我们提出了动态偏移模块，它可以自动适应每个激光雷达点在复杂的自动驾驶场景中的核函数，以便回归中心可以动态、高效且精确地偏移到正确的聚类中心。

为了使核函数可学习，我们首先考虑如何数学上定义一个可微的偏移操作。受到[99]的启发，如果迭代次数固定，播种点（即要聚类的点）上的偏移操作可以表示为矩阵操作。具体来说，偏移操作的一次迭代可以表述如下。记 $X \in \mathbb{R}^{M \times 3}$ 为M个播种点，X将通过偏移向量 $S \in \mathbb{R}^{M \times 3}$ 更新一次，公式为：

$$
X \leftarrow X + \eta S,
$$

其中η是缩放因子，在实验中设置为1。偏移向量S的计算是通过在X上应用核函数f，并且定义为 $S = f(X) - X$ 。

在各种核函数中，**平核**简单但对生成激光雷达点的偏移目标估计有效，如下介绍。应用平核的过程可以看作是在每个播种点中心放置一个特定半径（即带宽）的查询球，平核的结果是查询球内点的质量。数学上，平核 $f(X) = D^{-1}KX$ 由核矩阵 $K = (XX^T \leq \delta)$ 定义，它掩盖了每个播种点带宽内的点，对角矩阵 $D = \text{diag}(K_1)$ 表示播种点带宽内的点的数量。

有了可微分版本的偏移操作定义后，我们继续实现动态偏移的目标，为每个点适应核函数。每个播种点的最优带宽必须动态推断，以便核函数适应不同大小的实例。一个自然的解决方案是直接回归每个播种点的带宽，但如果与平核一起使用，则不是可微分的。尽管高斯核可以使直接带宽回归可训练，但它仍然不是最佳解决方案，如IV-A节所分析的。因此，我们采用加权多个带宽候选项的设计，以动态适应最优带宽。

动态偏移的一次迭代正式定义如下。如图2的下半部分所示，设置了  $l$  个带宽候选项  $L = \{\delta_1, \delta_2, ..., \delta_l\}$ 。对于每个种子点，通过对应于带宽候选项的  $l$  个平坦核计算出  $l$  个偏移目标候选项。然后，种子点通过学习权重  $W \in \mathbb{R}^{M \times l}$  来加权  $l$  个候选目标，动态决定最终的偏移目标，理想情况下这些目标最接近于聚类中心，通过应用多层感知器（MLP）和Softmax在主干特征上学习权重，使得  $\sum_{j=1}^{l} W[:, j] = 1$ 。上述过程和新的可学习核函数  $\hat{f}$  可以表述如下：

$$ \hat{f}(X) = \sum_{j=1}^{l} W[:, j] \odot (D_j^{-1} K_j X) $$

其中  $K_j = (X X^T \leq \delta_j)$  且  $D_j = \text{diag}(K_j 1)$ 。

明确了动态偏移的一次迭代后，动态偏移模块的完整流程，正式定义在算法1中，可以描述如下。首先，为了保持算法的效率，在  $M$  个事物点上执行最远点采样（FPS），为动态偏移迭代提供  $M'$  个种子点（第1-2行）。经过固定数量  $I$  的动态偏移迭代之后（第3-12行），所有种子点都已聚集到聚类中心。然后执行一个简单的启发式聚类算法，对聚集的种子点进行聚类，以获得每个种子点的实例ID（第13行）。最后，所有其他事物点找到最近的种子点，并将相应的实例ID分配给它们（第14-15行）。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/47d16a7326404f378c5a076178f77319.png" width="70%" /> </div>


由于实际上无法获得每个种子点的真实带宽，动态偏移模块的优化并不直观。损失函数必须鼓励种子点向没有真实值但可以由实例的真实中心  $C'_ {gt} \in \mathbb{R}^{M' \times 3}$  近似的聚类中心移动。因此，动态偏移第  $i$  次迭代的损失函数由真实中心  $C'_{gt}$  和第  $i$  次动态计算出的偏移目标  $X_i$  之间的曼哈顿距离定义，可以表述如下：

$$ l_i = \frac{1}{M'} \sum_{x=1}^{M'} \| X_i[x] - C'_{gt}[x] \|_1 $$

将  $I$  次迭代的所有损失相加，得到动态偏移模块的损失函数  $L_{ds}$ ：

$$ L_{ds} = \sum_{i=1}^{I} w_i l_i $$

其中  $w_i$  是不同迭代损失的权重，在实验中都设为1。

## C. 语义和实例分割的融合

通常，在全景分割中解决语义和实例预测之间的冲突是关键步骤之一。自下而上方法的优势在于所有预测实例ID的点必须属于事物类别，并且一个点不会被分配给两个实例。需要解决的唯一冲突是由实例分割的类别不可知方式引入的一个实例内部的语义预测不一致。我们使用的策略是多数投票。对于每个提议的实例，我们直接将内部最频繁的语义标签分配给该实例的所有点，以确保语义和实例分割结果之间的一致性。这种简单的融合策略不仅高效，还可以使用实例信息修正和统一语义预测。

## D. 4D光学激光雷达分割

基于上述提出的单帧激光雷达全景分割方法DS-Net，我们进一步将其扩展到4D全景激光雷达分割任务。

为了将单帧全景分割扩展到其4D对应物，需要在帧之间保持事物ID的一致性。换句话说，对于在多个帧中观察到的同一实例，目标是为它们分配相同的ID。简单的方法是将跟踪模块附加到实例分割分支，以关联预测的实例段从先前和当前帧。然而，这种简单堆叠模块的方式将不可避免地由于其依赖于分割质量而导致跟踪性能受损。此外，对于跟踪模块来说，很难充分利用连续激光雷达扫描提供的信息，因为它只处理裁剪后的局部观测。由于跟踪模块很难从未完整、稀疏的点云中提取出有区分度的特征，我们提出以更统一的方式来处理4D全景分割，以便充分利用连续激光雷达扫描的信息并隐式关联。

类似于4D-PLS[12]，我们采用SLAM系统估计的姿态来对齐和重叠连续的激光雷达扫描，形成4D数据体。然后设计了时间统一的实例聚类来以帧不可知的方式执行实例分割，其结果进一步与语义分割融合，形成最终的4D全景分割结果。这种在4D数据体上的统一分割避免了复杂后跟踪模块的需要。与此同时，信息提取过程完全具有时空意识，使其比“分割然后跟踪”的方法更有效。

*Temporally Unified Instance Clustering:* 为了确保事物ID的一致性，我们提出使用时间统一的实例聚类来替代显式的关联。这种聚类策略的目标是将几帧中同一实例的所有点联合聚类到一个单一的簇中。然后我们可以自然地将这些点分离到不同的帧中，并分配给它们相同的实例ID。为了将这种聚类策略适配到底向上的流水线中，我们需要修改中心回归步骤的目标以及随后的聚类模块。在流水线的单帧版本中，点级特征被用来回归实例的中心。然而，在多帧场景中，例如汽车和行人的位置会在帧间变化。因此，如果我们仍然遵循单帧版本的中心回归目标，同一个实例很可能因为高移动速度导致回归中心相隔太远而被聚类到多个簇中。为了避免这个问题，对于同一个实例，我们提出向多个帧重叠的点云中心回归，可以表述为：

$$
C_{gt}(id) = \text{center}\{p | p \in \cup_{i=0}^{t} g_{t+i}(id)\}
$$

其中  $p_{id}$  是具有事物ID id的点， $g_{t+i}(id)$  表示具有事物ID id且在帧  $t+i$  中的点集。调整后的中心回归之后，随后的聚类步骤在重叠的回归中心上执行。与[12]提出的4D体素聚类不同，我们的聚类过程不考虑每个点的帧时间戳，这意味着我们的方法与帧无关。理想情况下，来自几帧的同一个实例的所有点被聚类到一个单一的簇中，这符合提出的时间统一实例聚类的目标。为了将聚类策略集成到单帧版本DS-Net中，我们需要从主干网络获取连续几个激光雷达帧的点级特征。有两种可能的方式来实现这一点。第一种是在数据层面合并连续的激光雷达扫描。第二种是在主干网络之后立即合并每个单独帧的特征图。从计算效率的角度来看，第一种方法更有效。在下采样和体素化之后，处理多帧相当于主干特征提取部分的单帧输入。为了证明这一点，我们测试了当两帧合并时的GPU内存使用情况（示例来自典型的SemanticKITTI激光雷达扫描）。数据层面融合消耗大约5483 MB的内存，而特征层面融合则需要大约9966 MB。就性能而言，我们通过广泛的实验发现第一种策略也是一个更好的策略。因此，基于上述分析，我们提出了DS-Net的4D扩展版本，如下所示。

*4D Extension of DS-Net:* DS-Net的4D扩展版本（即4D-DS-Net）用于4D全景分割，如图4所示。使用SLAM算法[17]估计的自我姿态，我们对齐连续的激光雷达点云并将它们重叠起来，得到时间融合的激光雷达点云。帧t到t+i的时间融合激光雷达点云定义为：

$$
P_{t:t+i} = \{p | p \in P'_ t \cup ... \cup P'_{t+i}\}
$$

$$
P'_ {t+i} = ((P_ {t+i}R^{-1}_ {t+i} + T_{t+i}) - T_t)R_t
$$

 $R_{t+i}$  和  $T_{t+i}$  分别代表帧  $t+i$  的旋转矩阵和平移向量。语义分割分支为每个点预测语义标签，就像单帧版本一样。实例分割分支为每个点产生时间一致的ID，这是通过上述提出的时间统一实例聚类实现的。具体来说，前景点首先被回归到重叠实例的中心。然后，回归的中心通过我们提出的动态偏移网络以帧不可知的方式进一步聚类。这样一个统一的实例聚类步骤自然地关联了跨帧的同一个实例，并节省了跟踪算法的工作量。一旦我们确保了连续两帧中事物ID的一致性，实例ID就可以通过重叠帧传播到整个序列中。具体来说，在两连续4D体素的情况下，一个重叠帧介于两个连续的4D体素之间。重叠帧将有两组预测的实例，我们计算IoU作为关联得分。对于超过0.5的关联得分，两个实例匹配。通过这种方式，实例ID被传播到整个序列中，从而得到最终的4D全景分割结果。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/a3a8dd48b5664008874e1f3d51e2ecb7.png" width="70%" /> </div>


# IV. 实验

我们在两个大规模数据集上进行了实验：SemanticKITTI [17] 和 Panoptic nuScenes [18], [19]。此外，我们还评估了我们在SemanticKITTI上的4D全景分割的扩展。

SemanticKITTI: SemanticKITTI数据集 [11] 是第一个提出基于激光雷达全景分割挑战的数据集，并提供了基准。4D-PLS [12] 通过引入新颖的4D全景分割任务进一步扩展了基准。SemanticKITTI包含23,201帧用于训练，20,351帧用于测试。有28个注释的语义类别，为基于激光雷达的全景分割任务重新映射到19个类别，其中8个类别是事物类别，11个类别是物质类别。每个点都被标记有一个语义标签和一个时间上一致的实例ID，如果点属于物质类别，则将其设置为0。

Panoptic nuScenes: Panoptic nuScenes数据集 [19] 是一个基于nuScenes [18] 构建的大规模基于激光雷达全景分割数据集。该数据集提供了1000个场景，包含32个语义类别和300k个实例。

*基于激光雷达全景分割的评估指标：* 如 [11] 中定义的，基于激光雷达全景分割的评估指标与 [9] 中定义的图像全景分割相同，包括全景质量（PQ）、分割质量（SQ）和识别质量（RQ），在所有类别上计算。对于每个类别，PQ、SQ 和 RQ 定义如下：

$$
\begin{align*}
\text{SQ} &= \frac{\sum_{(i,j) \in T} P \text{ IoU}(i, j)}{|TP|} \\
\text{RQ} &= \frac{|TP|}{|TP| + \frac{1}{2}|FP| + \frac{1}{2}|FN|} \\
\text{PQ} &= \text{SQ} × \text{RQ}
\end{align*}
$$

上述三个指标也分别对事物和物质类别进行计算，得出PQTh、SQTh、RQTh和PQSt、SQSt、RQSt。PQ†通过交换每个物质类别的PQ为其IoU，然后平均所有类别来定义。此外，还使用平均交并比（mIoU）来评估语义分割子任务的质量。

4D全景激光雷达分割的评估指标：
先前的视频全景分割工作 [10], [12], [37] 提出了几种指标。在其中，我们选择使用LSTQ（激光雷达分割和跟踪质量）[12] 作为4D全景分割的评估指标，定义如下：

$$
\begin{align*}
\text{LSTQ} =& (\frac{1}{C} \sum_{c=1}^{C} \text{IoU}(c) \\
&\times \frac{1}{T} \sum_{t \in T} \sum_{s \in S} \text{TPA}(s, t) \text{IoU}(s, t) \\
&\times \frac{1}{|gtid(t)|})^\frac{1}{2}
\end{align*}
$$

其中Srmcls和Srmass分别反映分割和跟踪质量。TPA（真正例关联）定义为TPA(i, j) = |pr(i) ∩ gt(j)|，表示预测为i的点与具有ID j的真实点之间的交集数量。

*骨干实现细节：* 按照PCSeg [105]和MMDetection3D [106]，对于两个数据集，每个输入点被表示为一个四维向量，包括XYZ坐标和强度。骨干网络在圆柱坐标系下将单个帧体素化为480×360×32体素。对于真实实例中心，我们不使用注释的3D边界框的中心。相反，它通过其紧盒子的中心近似，这比不完整点云的质量中心提供了更好的近似。

*动态偏移实现细节：* 带宽候选项设置为0.2、1.7和3.2，用于两个数据集。迭代次数设置为4，用于两个数据集。我们使用学习率为0.002、周期数为50、批量大小为4，在四个Geforce GTX 1080Ti上训练网络。动态偏移模块仅需要3-5小时就可以在预训练的骨干网络上进行训练。

*4D-DS-Net实现细节：* 两个连续的激光雷达扫描被对齐和重叠，用于4D全景分割的训练和推理。动态偏移模块中的FPS下采样点数设置为20000。其他超参数与其单版本对应物相同。对于DS-Net和4D-DS-Net，我们首先使用语义分割损失预训练骨干，然后添加实例分支进行微调。最后，固定先前网络训练动态偏移模块。

## A. 消融研究
*整体框架消融研究：* 为了研究提出的模块的有效性，我们依次将多数投票模块和动态偏移模块添加到简单的全景分割骨干中。简单的骨干由圆柱骨干、语义和实例分支以及均值漂移聚类算法组成。相应的PQ和PQTh在图5(a)中报告，表明两个模块都为DS-Net的性能做出了贡献。新颖的动态偏移模块主要提高了实例分割的性能，这由PQTh表明，在验证分割中，DS-Net比骨干（带融合模块）高出3.2%。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/c0eaca66bda84a82b74b46e835d87e35.png" width="70%" /> </div>


*聚类算法的消融：* 为了验证我们之前对聚类算法的分析，我们将动态移动模块替换为其他四种广泛使用的启发式聚类算法： BFS、DBSCAN、HDBSCAN和平均位移。结果如图5(b).所示与我们在第III-B节中的分析一致，基于密度的聚类算法（如BFS，DBSCAN，HDBSCAN）在PQ方面表现不佳

*消融聚类算法研究：* 为了验证我们之前对聚类算法的分析，我们将动态偏移模块替换为四种其他广泛使用的启发式聚类算法：BFS、DBSCAN、HDBSCAN 和均值漂移。结果如图 5(b) 所示。与我们在第三节 B 中的分析一致，基于密度的聚类算法（例如 BFS、DBSCAN、HDBSCAN）在 PQ 和 PQTh 方面表现不佳，而均值漂移在启发式算法中取得了最佳结果。此外，我们的动态偏移模块优于所有四种启发式聚类算法。

*带宽学习风格的消融：* 在动态偏移模块中，如第三节 B 所述，直接对每个点的带宽进行回归是很自然的事情。然而，如图 5(c) 所示，在这种情况下直接回归难以优化，因为学习目标并不直接。确定每个点的最佳带宽是困难的，因此直接对回归带宽进行监督是不切实际的。因此，网络更容易从几个带宽候选项中进行选择和组合。

*不同骨干选择的消融：* 为了证明动态偏移模块可以应用于不同的骨干，我们报告了普通骨干和 DS-Net 的矩形卷积版本的性能，如图 5(d) 所示。在 SemanticKITTI 的验证集上，矩形卷积版本和圆柱卷积版本的普通骨干在 PQ 方面都实现了 2.3% 的提升，这表明所提出的动态偏移模块也可以与其他骨干一起工作。

## B. 在SemanticKITTI上的比较

在表II和III中，我们总结了SemanticKITTI上所有可用的结果，并用场地和年份进行注释，以便进行清晰和彻底的比较。DS-Net*报告了将骨干从Cylinder3D更改为SPVCNN [55]的结果。结果表明，原始DS-Net在同期工作中取得了优越的性能。迁移到更强的骨干后，DS-Net在验证和测试分割中仍然取得了竞争性的成绩，显示出所提出的动态偏移网络的有效性。值得注意的是，PointGroup [14]在激光雷达点云上表现不佳，表明室内解决方案不适用于具有挑战性的激光雷达点云。此外，使用混合骨干的方法，例如GP-S3Net和Panoptic-PHNet，比单一骨干的方法有更好的整体性能。这一观察结果可能为未来研究人员在选择骨干时提供强有力的线索。这也清楚地表明，混合骨干结构设计也是一个有前途的未来方向。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/70c9cc735d9a4fa8af40973fac0c6feb.png" width="70%" /> </div>
<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/a28a48ae888645ec81cc9e1e18782e3b.png" width="70%" /> </div>


我们在图6中展示了我们结果的一些可视化。左侧显示了带有多数投票模块的裸骨干的结果。中间的显示了DS-Net的结果，右侧是真实情况。我们的DS-Net能够正确处理形状和密度复杂的实例，而骨干方法在这些情况下倾向于过度或欠分割。有关更多可视化，请参考补充材料。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/691d1c2f0dac4d338c32d3fb57e6bd57.png" width="70%" /> </div>


## C. 在Panoptic Nuscenes上的比较

我们在表IV和V中报告了Panoptic nuScenes的验证集和测试集上的结果。我们收集了所有报告该数据集结果的方法，并将它们用场地和年份进行注释，以便进行完整的比较。同样，DS-Net*报告了将骨干更改为SPVCNN [55]的结果。在同期工作中，我们的方法取得了最好的结果。诚然，我们的结果与更近期的解决方案之间仍然存在差距。表V中的前三条线是强大的分割和检测方法的组合。Panoptic-PHNet和LidarMultiNet甚至在PQ得分上超过了80%。这些方法要么是自上而下的方法，要么大量借鉴了检测解决方案的结构，例如中心热图。它们的出色表现表明，对于像panoptic nuScenes这样的数据集，其中激光雷达扫描更稀疏，纯基于聚类的方法比自上而下的方法有更多的缺点。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/a71289dd93e94c1da2d9b7565c7ea317.png" width="70%" /> </div>
<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/678c5a41e5c94cf68b9d8aa45850514c.png" width="70%" /> </div>


## D. 4D全景激光雷达分割结果

*比较方法：* 由于这项任务相当新，我们选择与提出这项任务的第一项工作 [12] 进行比较，记为‘4D-PLS’，两项近期的工作4D-StOP [76], CIA [85] 以及几个‘语义分割 + 3D目标检测 + 跟踪’组装的基线方法。此外，我们还在实例分割分支上添加了一个跟踪模块 [10] 来构建一个基线方法，记为‘DS-Net + Tracking’。如第三节D所讨论的，我们还实现了DS-Net之上的特征图融合，即‘DS-Net + Feat. Fus.’。具体来说，我们在骨干网络提取的连续激光雷达帧上执行对齐3D特征图的最大池化操作。然后，融合后的特征图被送入语义和实例分支。我们将我们提出的方法称为‘4D-DS-Net’。4D-DS-Net*表示将骨干更改为Cylinder3D++ [15]。

*评估结果：* 如表VI和VII所示，我们提出的方法在验证集和测试集中的主要指标LSTQ方面超过了所有组装的基线方法和最先进的方法。‘4D-DS-Net’在验证集上比‘DS-Net + Tracking’在LSTQ方面高出2.1%，这证明了如第三节D所述，简单地堆叠模块很难充分利用时间信息。注意，“DS-Net + Feat. Fus.”有更高的关联得分Sassoc，但分割得分Scls比4D-DS-Net差。我们认为可能有两个原因。a) 特征级融合从每个激光雷达扫描单独提取特征，使得实例级特征提取不易受到如问题Q1中讨论的“拖尾”问题的影响。这导致更好的4D实例聚类，有利于关联得分Sassoc。b) 4D-DS-Net采用数据级融合策略，意味着几个连续的激光雷达帧被对齐、重叠，并作为单个扫描送入特征提取主干网络。原本部分观察到的事物和背景现在有多个视角的信息，这降低了语义分割的难度。因此，数据级融合策略有利于分割得分Scls。我们还对SemanticKITTI[12][17]验证集和测试集上用于4D全景分割的帧数进行了消融研究，如表VI和表VII所示。有趣的是，“3-scan”版本在三种变体中取得了最佳结果。我们认为这是因为，随着更多扫描的重叠，“拖尾”问题对4D体素中的实例聚类提出了更多挑战。然而，更多的扫描会给网络提供更密集的点云和对场景的更多观察，这有利于语义理解。因此，“3-scan”是这两个因素之间的平衡点。我们还在主要手稿中包含了这个讨论。然而，与‘4D-DS-Net’相比，‘DS-Net + Feat. Fus.’在内存和计算开销上的数量仍然证明了我们对数据级融合的偏好。有关定性评估，请参阅补充材料。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/164df7bc66474878af396446b614f7d7.png" width="70%" /> </div>
<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/087b1c4200434872a4032236b5113def.png" width="70%" /> </div>


*4D泛视分割改进了单帧PQ评估：* 我们还使用DS-Net的4D版本在SemanticKITTI的验证集上评估了单帧指标。如表VIII所示，4D版本的DS-Net在PQ方面比单帧DS-Net高出1.8%。这表明时间信息可以大大丰富由主干网络提取的语义信息，从而提高整体性能。改进的单帧分割质量也解释了在4D全景激光雷达分割任务中更好的性能。此外，'4D-DS-Net'在PQ方面也比'DS-Net + Feat. Fus.'高出0.9%，这表明数据级融合比简单的特征级融合更优越。当然，可以设计更复杂的特征融合，并有可能超过数据级融合。但鉴于内存和计算开销最小，数据级融合是这里的首选。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/b851d0bf82e6455eacd4218820a8f498.png" width="70%" /> </div>


## E. 进一步分析

**对参数设置的鲁棒性**：如表IX所示，我们为独立训练设置了六组带宽候选项，并报告了相应的结果。稳定的结果表明，只要所选的带宽候选项与实例大小相当，DS-Net对不同的参数设置就具有鲁棒性。与之前需要一些超参数搜索的启发式聚类算法不同，DS-Net能够自动调整以适应不同的实例大小和点分布，并保持稳定的聚类质量。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/845cf8bf56e54af4ab0efbff57e6e40f.png" width="70%" /> </div>


**可解释的学到的带宽**：通过对每个点的带宽候选项按学到的权重平均，可以相应地近似每个点的学到的带宽。不同类别的平均学到的带宽显示在图7中。可以看到，平均学到的带宽大致与相应类别的实例大小成比例，这与动态偏移能够动态调整不同实例大小的预期一致。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/5ed213e00c684ab4a06e9badce0cdcc0.png" width="70%" /> </div>


**动态偏移迭代的可视化**：如图8所示，黑点是包括行人、骑自行车者和汽车在内的不同实例的原始点云。播种点用光谱颜色着色，其中红色点代表更高的学到的带宽，蓝色点代表更低的学到的带宽。远离实例中心的播种点倾向于学习更高的带宽以快速收敛。而学习到的回归点倾向于有更低的带宽以保持其位置。经过四次迭代后，播种点围绕实例中心汇聚。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/e36791c7fced4b0eb3c8fbd0c0487e14.png" width="70%" /> </div>


**不同迭代的学到的带宽**：不同迭代的平均学到的带宽显示在图9中。正如预期的那样，随着迭代轮数的增加，同一实例的点更紧密地聚集在一起，通常需要更小的带宽。经过四次迭代后，大多数类别的学到的带宽已降至0.2，这是它们能降到的最低值，意味着四次迭代足以让事物点汇聚到聚类中心，这进一步验证了上一段中所做的结论。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/c61e817cb46c4e34bf3ca08fd46aee80.png" width="70%" /> </div>


**推理时间分析**：DS-Net中不同模块在nuScenes上的推理时间报告在表X中。动态偏移模块每帧需要33.1毫秒，这归因于下采样和GPU加速的矩阵运算。核运算之后，播种点大部分已收敛。因此，最终聚类步骤的时间可以忽略不计。与均值漂移相比，后者需要190.3毫秒，所提出的动态偏移模块是高效的并且表现更好。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/1eb455aaf5fb4ed5bf601bf9d3607139.png" width="70%" /> </div>


**失败案例**：快速移动的实例会导致在4D体素中与长拖尾重叠的点，这增加了实例聚类的难度。这可能导致在4D体素中的过度分割，从而导致在跟踪过程中错误地更改实例ID。除了拖尾问题外，困难类别也容易错误分割。有关定性评估，请参阅补充材料。

# V. 结论

为了提供自动驾驶的全面感知，我们提出了一个统一的3D和4D基于激光雷达的全景分割框架。为了应对激光雷达点云非均匀分布带来的挑战，我们提出了新颖的DS-Net，它专门设计用于有效的激光雷达点云全景分割。新颖的动态偏移模块能够适应不同密度和大小的实例的回归中心。通过构建4D数据体并对其执行动态偏移聚类，我们自然地将DS-Net的单帧版本扩展到4D版本。通过在两个大规模数据集上的广泛实验，我们展示了DS-Net和4D-DS-Net的竞争性能。进一步的分析显示了动态偏移模块的鲁棒性和学到的带宽的可解释性。

# 声明

本文内容为论文学习收获分享，受限于知识能力，本文队员问的理解可能存在偏差，最终内容以原论文为准。本文信息旨在传播和学术交流，其内容由作者负责，不代表本号观点。文中作品文字、图片等如涉及内容、版权和其他问题，请及时与我们联系，我们将在第一时间回复并处理。
