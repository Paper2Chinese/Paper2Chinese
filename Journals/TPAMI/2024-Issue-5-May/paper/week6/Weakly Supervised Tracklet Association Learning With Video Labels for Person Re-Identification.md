# 题目：[Weakly Supervised Tracklet Association Learning With Video Labels for Person Re-Identification](https://ieeexplore.ieee.org/document/10371780/)  
## 基于视频标签的弱监督Tracklet关联学习用于人的重新识别
**作者：Min Liu; Yuan Bian; Qing Liu; Xueping Wang; Yaonan Wang** 

**源码链接：** 
****

# 摘要

监督式人物再识别（re-id）方法需要昂贵的手动标记成本。尽管无监督re-id方法可以减少标记数据集的需求，但这些方法的性能低于监督式替代方案。最近，一些基于弱监督学习的人体re-id方法已被提出，这是监督式和无监督式学习之间的一种平衡。然而，这些模型中的大多数需要另一个辅助的全监督数据集或忽略了嘈杂轨迹的干扰。为了解决这个问题，我们提出了一个仅利用视频标签的弱监督轨迹关联学习（WS-TAL）模型。具体来说，我们首先提出了一个内部包轨迹歧视学习（ITDL）项。它可以通过为包中的每个人像分配伪标签来捕捉个人身份和图像之间的关联。然后，通过过滤嘈杂的轨迹，利用获得的关联学习每个人的判别特征。基于此，提出了一个跨包轨迹关联学习（CTAL）项，通过挖掘可靠的正轨迹对和硬负对来探索包之间的潜在轨迹关联。最后，这两个互补的项被联合优化以训练我们的re-id模型。在弱标记数据集上的广泛实验表明，WS-TAL在MARS和DukeMTMC-VideoReID数据集上分别达到了88.1%和90.3%的rank-1准确率。我们模型的性能以大幅度超过了现有的弱监督模型，甚至超过了一些全监督的re-id模型。

# 关键词

- 弱监督学习
- 人物再识别
- 视频标签
- 轨迹关联学习

# I. 引言

人员再识别是图像检索的一个子任务，它从多个不重叠的摄像头中检索目标人物。得益于深度神经网络（DNNs）的广泛应用[1][2][3][4][5]，近年来基于DNNs的再识别方法[6][7][8][9][10][11][12][13][14]取得了显著进展。然而，这些方法在很大程度上依赖于昂贵的手动标签。为了减少对标记数据的大量需求，一些近期的研究集中在无需任何标记信息的无监督人员再识别上。但是，由于缺乏手动标签指导，这些方法[15][16][17][18][19][20]的性能通常不如监督式替代方案。

为了平衡标记数据的需求和识别性能，近年来有几项工作[7][21][22][23]集中在弱监督人员再识别上。弱设置指的是用视频标签标注再识别数据。注释者只需要粗略地看一眼视频中出现的人，而不需要精确地给每个人员轨迹标记身份标签。具体来说，在标注数据集的过程中，可以通过使用行人检测和跟踪算法自动获得人员轨迹，但是当算法不完美时，会引入一些嘈杂的轨迹。对于弱标记数据，注释者只需要将所有检测到的轨迹分组到一个包中，并为每个视频标记一个视频标签，如图1(b)所示。相反，对于强标记数据，注释者需要去除嘈杂的检测和跟踪结果，并精确地关联人员轨迹及其身份，如图1(a)所示。显然，与强标签相比，视频标签以较低的成本和更高级别的抽象获得。然而，监督信号较弱。


<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/9233f0f421c348518ba6975a450bcf5d.png#pic_ center" width="70%" />
</div>

为了解决这个问题，一些近期的工作已经提出，从弱标记数据集中学习再识别模型[7][21][22][23]。然而，[21][22]除了弱标签外，还需要每个身份的一个强标签，而[7][23]在模型训练期间忽略了轨迹信息。在本文中，我们通过仅利用视频标签，提出了一个弱监督轨迹关联学习（WS-TAL）模型来进行人员再识别。 

具体来说，所提出的WS-TAL模型由两个互补的项组成：包内轨迹歧视学习（ITDL）项和跨包轨迹关联学习（CTAL）项。首先，所提出的ITDL项可以通过为包中的每个人像分配伪标签来捕获个人身份和图像之间的关联。伪标签的每个帧指示视频中最有可能对应的身份。根据分配的标签和轨迹信息，我们可以为每个视频中的每个出现的身份选择一个有效的轨迹来学习每个人的特征，这可以使我们学习到的模型受嘈杂轨迹的影响较小。此后，在伪标签的监督下，可以学习每个人的判别特征。ITDL仅考虑每个包隔离，没有探索包间的轨迹关联。我们通过引入跨包轨迹关联学习（CTAL）项来解决这个问题，该项通过挖掘可靠的正轨迹对和硬负对来探索包之间的潜在轨迹关联。具体来说，在两个包之间的轨迹对中，选择具有最高相似性值的轨迹对作为可靠的正对。在此之上，可以通过在另一个包中寻找具有第二高相似性值的轨迹来为正对中的每个样本找到硬负轨迹。可以注意到，在我们的框架中，在ITDL的过滤过程之后，每个包中只有一个人轨迹对应于一个身份。因此，所选择的负轨迹是硬负的。CTAL试图将硬负轨迹对相互推开。最后，我们联合优化这两个互补的项，以在弱标记数据集上学习有效的人员再识别模型。

此外，我们通过图1(c)和(d)所示的两种不同策略评估所提出的WS-TAL：轨迹对轨迹（T2T）再识别，即检索与目标轨迹在同一身份下的轨迹；轨迹对视频（T2V）再识别，即找到目标人物出现的短视频片段。

我们的贡献总结如下：
- 我们为只需要视频标签的弱监督人员再识别任务制定了一个弱监督轨迹关联学习（WS-TAL）模型。
- 我们为弱标记数据提出了包内轨迹歧视学习项，能够有效地消除嘈杂的轨迹，并学习每个人的判别特征。
- 我们引入了跨包轨迹关联学习项，通过挖掘正轨迹对以及相应的硬负对来探索包之间的潜在轨迹关联。
- 广泛的实验表明，我们的模型WS-TAL在MARS数据集上达到了88.1%的rank-1准确率，显著优于比较的现有弱监督模型，同样，在DukeMTMC-VideoReID数据集上为T2V再识别达到了90.3%。甚至我们的模型在某些完全监督模型上也表现出色，例如，在MARS数据集上与[24]相比，rank-1准确率提高了5.2%（即78.7%对83.9%）。


# III. 方法

在本节中，我们提出了一种基于视频标签的弱监督轨迹关联学习模型，用于重识别任务。首先，我们陈述问题设置并正式定义符号。然后，引入了袋内轨迹判别学习项（Intra-Bag Tracklet Discrimination Learning, ITDL）和跨袋轨迹关联学习项（Cross-Bag Tracklet Association Learning, CTAL），分别用于学习每个袋中每个人的判别特征，并探索袋间的轨迹关联。最后，我们展示了我们方法的训练和评估策略。在图 2 中展示了我们提出的方法的框架，并且值得注意的是，我们的模型仅在模型训练期间利用视频标签。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/7c196be4e114457fb91df625b224e2ff.png#pic_ center" width="70%" />
</div>

## A. 问题陈述和注释

在我们的弱监督人物重识别设置中， $N$ 个视频片段出现 $C$ 种人物身份构成弱监督训练集；每个视频片段都标注有一个相应的视频标签，指示这个视频中出现了哪些身份。在每个视频片段中自动检测到的人物轨迹[57]、[58]被分组到一个袋子中，代表这个视频。因此，训练数据表示为 $D = \{B_ n, y_ n\}_ {n=1}^{N}$ 。假设所有轨迹包括 $K_ n$ 个人图像由第 $n$ 个视频片段生成 $B_ n = \{I_ {n,k}\}_ {k=1}^{K_ n}$ 。第 $t$ 个轨迹可以表示为 $s_ {n,t} = \{I_ {n,k}\}_ {k \in N(s_ {n,t})}$ ，其中 $N(s_ {n,t})$ 是轨迹帧的索引集。紧随主干网络之后的全连接层被视为一个特征提取器，用于提取袋子 $B_ n$ 中人物图像的相应特征表示，我们将它们以特征矩阵 $X_ n \in \mathbb{R}^{d \times K_ n}$ 的形式堆叠。 $y_ n \in \{0, 1\}^C$ 是一个标签向量，包含 $C$ 个身份。如果第 $c$ 个身份出现在第 $n$ 个视频片段中，则 $y_ {c,n} = 1$ ，否则 $y_ {c,n} = 0$ 。注意，检测到的轨迹和视频标签中的身份之间的确切对应关系是未知的。

在测试探针集中，每个人轨迹被视为一个查询样本。在图库集中，定义了两种不同的设置如下：轨迹到轨迹（Tracklet-to-Tracklet, T2T）重识别的目的是检索图库集中与目标轨迹相同人身份的轨迹；每个图库样本是一个具有单一人身份的轨迹，这与一般基于视频的人物重识别设置相同；轨迹到视频（Tracklet-to-Video, T2V）重识别的目的是找到目标人物出现的视屏片段。每个测试图库样本是一个袋子，设置与训练集相同。

## B. 袋内轨迹识别学习

在实际情况中，每个袋子是通过行人检测和跟踪算法[57]、[58]自动获得的，可能包含一些噪声轨迹。具体来说，我们将这些噪声轨迹分为三类：(1) 由于检测算法的不准确，可能存在一些没有人物的背景图像（轨迹）。(2) 遮挡可能导致错误的人物跟踪，使得一个人轨迹被追踪成多个轨迹（例如轨迹碎片问题[39]）。(3) 注释者可能因为人物只在视频中出现很短时间而错过了一个人，导致视频标签不可靠。

噪声轨迹在实际情况中是不可避免的，因此在噪声轨迹的干扰下学习判别性人物特征是一个具有挑战性的任务。本文提出了一种袋内轨迹判别学习（ITDL）方法来解决这个问题。它首先为袋子中的每个人物图像分配一个伪标签，以捕获人物身份和图像之间的关联。然后我们通过引入基于分配的伪标签和轨迹信息的身份率来过滤掉噪声轨迹，并为每个袋子中出现的每个身份获取有效的轨迹。最后，在伪标签的监督下学习每个人的判别特征。

1)**Pseudo Labels Assignment**: 每个弱标记样本由多个具有多个身份的人物轨迹组成。由于标签信息的弱性质，帧和视频标签之间的关联是未知的。为了捕获每个袋子中人物图像和身份之间的关联，我们利用预测概率和视频标签为每个帧分配伪标签。

具体来说，对于第 $n$ 个袋子 $B_ n$ ，我们将提取的特征 $X_ n \in \mathbb{R}^{d \times K_ n}$ 输入到分类层 $f(\cdot, \theta)$ ，其中 $\theta$ 是该层的参数，然后通过 softmax 函数对袋子中的每个帧进行操作，我们可以获得帧 $I_ {n,k}$ 的身份预测概率 $\hat{p}_ {n,k} \in \mathbb{R}^C$ 。对帧身份预测概率 $\hat{p}_ {n,k}$ 和真实视频标签 $y_ n \in \mathbb{R}^C$ 进行逐元素乘积操作，将具有最高乘积值的身份 $\hat{y}_ {n,k}$ 分配给该人物图像作为伪标签。请注意，每个帧的伪标签表示视频中最可能出现的相应身份。可以定义如下：

$$
\hat{p}_ {n,k} = \text{softmax}(f(X_ {n,k}; \theta))
$$

$$
\hat{y}_ {n,k} = \arg\max [y_ n \odot \hat{p}_ {n,k}]
$$

其中 $\odot$ 表示逐元素乘积操作。分配的伪标签 $\hat{y}_ {n,k}$ 可以指导每个人的判别特征学习。对于每次迭代，伪标签将动态更新，即每个训练迭代中每个帧的伪标签将被重新分配。因此，随着模型的训练，伪标签的可靠性将逐渐提高。

2)**Intra-Bag Discriminative Feature Learning**: 为了减少噪声轨迹对我们学习模型的影响，我们利用每个轨迹的身份率来选择每个视频中每个出现身份的有效轨迹，并通过动态更新每帧的伪标签来训练人物重识别模型。轨迹 $s_ {n,t}$ 对应身份 $c_ {n,i}$ 的身份率 $r$ 通过使用分配的伪标签和轨迹信息计算得出：

$$
\delta(I_ {n,k}, c_ {n,i}) =
\begin{cases}
1, & \text{if } \hat{y}_ {n,k} = c_ {n,i} \\
0, & \text{otherwise}
\end{cases}
$$

$$
r(s_ {n,t}, c_ {n,i}) = \frac{\sum_ {k \in N(s_ {n,t})} \delta(I_ {n,k}, c_ {n,i})}{|N(s_ {n,t})|}
$$

其中 $c_ {n,i} \in \{ \arg[y_ {n} == 1] \} (c = 1, 2, 3, ..., C)$ 表示第 $n$ 个视频片段中第 $i$ 个标记的身份， $k \in N(s_ {n,t})$ 。它表示轨迹 $s_ {n,t}$ 属于身份 $c_ {n,i}$ 的概率，如图 3 所示。在一个袋子中，我们将身份率最高的轨迹视为每个出现身份的有效轨迹。过滤过程后，一个身份标签将对应一个有效轨迹。因此，噪声轨迹将被过滤掉，不会干扰我们模型的训练。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/04dfbfbbb0d74ddcb8e972e89d3b19a3.png#pic_ center" width="70%" />
</div>

通过动态更新每一帧的伪标签，随着训练迭代次数的增加，有效的轨迹线会越来越可靠。

之后，对于每个有效的tracklet，我们可以通过利用每个袋子中的人图像的生成的伪标签，以完全监督的方式学习discretion-inative特征。我们采用交叉熵损失函数来优化网络参数。它可以计算为：

$$
l_ {c e}=-\sum_ {c=1}^{C} \mathbb{1}\left(c==\hat{y}_ {n, k}\right) \log \hat{p}_ {n, k}^{c}
$$

其中，I表示返回1/0的指示函数，该指示函数指示来自有效轨迹的帧In的伪标签是否是cth标识。给定具有来自有效轨迹的总共K个人物图像的训练包，我们计算ITDL损失为：

$$
L_ {I T D L}=\frac{1}{K} \sum_ {k=1}^{K} l_ {c e}^{k}
$$

通过最小化LITDL，该模型可以学习的人的图像的鉴别特征。

## C. 跨袋轨迹关联学习

基于之前学习到的人物图像特征，可以通过对同一有效轨迹中的人物图像特征执行均值池化操作来获得袋子 $B_ n$ 中第 $t$ 个有效轨迹的特征 $S_ {n,t}$ 。ITDL 考虑每个袋子是孤立的，但并未考虑袋子之间的关联。为了弥补这一不足，我们引入了跨袋轨迹关联学习（CTAL）项，以探索袋子之间的潜在轨迹关联。

对于弱标记数据，同一个人出现在多个视频片段（袋子）中是一种常见现象。因此，我们选择每批至少有一个共同人物身份的袋子对来训练模型。具体来说，给定一对袋子 $B_ u$ 和 $B_ v$ ，分别包含 $T_ u$ 和 $T_ v$ 个有效轨迹，来自不同的摄像机视角，至少有一个共同的人物身份。我们假设两个袋子 $B_ u$ 和 $B_ v$ 之间具有最高相似性值的轨迹对 $(S_ {u,a}, S_ {v,b})$ 具有相同的身份，并将它们视为可靠的正样本轨迹对。定义如下：

$$
(a, b) = \arg\min_ {i \in \{1,2,3,...,T_ u\}, j \in \{1,2,3,...,T_ v\}} D(S_ {u,i}, S_ {v,j})
$$

$$
d_ {\text{pos}}^{B_ u,B_ v} = D(S_ {u,a}, S_ {v,b})
$$

其中 $a$ 和 $b$ 是正样本轨迹对中 $S_ {u,a} \in B_ u$ 和 $S_ {v,b} \in B_ v$ 的索引，相似性值通过欧几里得距离 $D(\cdot, )$ 计算，此值 $d_ {\text{pos}}^{B_ u,B_ v}$ 被视为 $B_ u$ 和 $B_ v$ 之间的正样本值。此外，我们基于得到的可靠正样本轨迹对 $(S_ {u,a}, S_ {v,b})$ 从袋子对 $B_ u$ 和 $B_ v$ 中挖掘硬负样本轨迹。具体来说，对于正样本轨迹 $S_ {u,a} \in B_ u$ ，我们在另一个袋子 $B_ v$ 中找到具有第二高相似度的硬负样本轨迹，同样地，对于 $S_ {v,b}$ 也进行同样的操作。因为过滤掉噪声轨迹后，每个袋子中每种身份只有一个轨迹，所以获得的对应对将是硬负样本。 $B_ u$ 和 $B_ v$ 之间的负样本值定义为：

$$
d_ {\text{neg}}^{S_ {u,a},B_ v} = \min_ {j \in \{1,2,...,T_ v\} \land j \neq b} D(S_ {u,a}, S_ {v,j})
$$

$$
d_ {\text{neg}}^{S_ {v,b},B_ u} = \min_ {i \in \{1,2,...,T_ u\} \land i \neq a} D(S_ {u,i}, S_ {v,b})
$$

$$
d_ {\text{neg}}^{B_ u,B_ v} = \frac{d_ {\text{neg}}^{S_ {u,a},B_ v} + d_ {\text{neg}}^{S_ {v,b},B_ u}}{2}
$$

因此，我们通过以下方式为每对袋子制定 CTAL 损失：

$$
l_ {\text{ctal}}(B_ u, B_ v) = -\log \frac{e^{-d_ {\text{pos}}^{B_ u,B_ v}}}{e^{-d_ {\text{pos}}^{B_ u,B_ v}} + e^{-d_ {\text{neg}}^{B_ u,B_ v}}}
$$

给定一个包含 $N_ b$ 个袋子对的训练批次，CTAL 损失可以表示为：

$$
L_ {\text{CT AL}} = \frac{1}{N_ b} \sum_ {u,v} l_ {u,v}^{\text{ctal}}
$$

其中 $(u, v)$ 是批次中袋子对的索引。通过最小化 $L_ {\text{CT AL}}$ ，模型将推动硬负样本轨迹对彼此远离，并将正样本对拉近。

## D. 模型训练和测试

总结来说，我们联合优化总损失 $L(x, w)$ ，包括 ITDL 损失和 CTAL 损失，以学习我们提出的 WS-TAL 模型，定义如下：

$$
L(x, w) = \lambda_ 1 L_ {\text{IT DL}} + \lambda_ 2 L_ {\text{CT AL}} + \|w\|^2
$$

其中 $\lambda_ 1$ 和 $\lambda_ 2$ 是超参数，分别控制模型训练阶段 ITDL 和 CTAL 的相对重要性。 $\|w\|^2$ 是权重的 L2 正则化。在第 IV-H 节中，讨论了每个部分对模型性能的贡献。

在测试阶段，根据我们的目标，我们通过两种不同的策略评估提出的方法：

1)**T2V 重识别**：测试探针集中的袋子 $B_ p$ 由袋子中所有图像帧的均值池化特征 $S_ p$ 表示， $X_ {g,k}$ 表示第 $g$ 个测试图库袋子中第 $k$ 个帧的特征。然后，袋子 $B_ p$ 和袋子 $B_ g$ 之间的距离定义如下：

$$
d_ {B_ p,B_ g} = \min_ {D(S_ p, X_ {g,1}), ..., D(S_ p, X_ {g,K_ g})}
$$

2)**T2T 重识别**：目标是检索图库集中与给定探针轨迹相同身份的轨迹。它与一般基于视频的人物重识别目标相同，因此测试图库集由人物轨迹组成。按照一般人物重识别设置评估 T2T 重识别的性能。

# IV. 实验

本节将展示我们提出的基于弱监督学习的人物重识别模型（WS-TAL）的实验评估。实验分为几个部分：首先，我们介绍了使用的数据集和评估协议；其次，我们提供了实现细节；然后，我们将我们的方法与现有的最先进方法进行了比较；最后，我们进行了超参数分析、鲁棒性分析，并在SYSU-30K数据集上测试了我们的方法。

## A. 数据集和评估协议

**弱标记数据集**：所有实验都在两个弱标记数据集上进行，即弱标记MARS（WL-MARS）和弱标记DukeMTMC-Video-ReID（WL-DukeV）。这两个数据集是基于两个现有的大规模基于视频的人物重识别数据集MARS和DukeMTMC-VideoReID生成的。然而，这些数据集的原始视频不可用，因此为了模拟真实场景，我们按照以下方式形成弱标记数据集：我们首先在同一个摄像机中随机选择3-6个具有不同身份的人物轨迹来形成一个袋子；然后，用所选轨迹的标签对其进行标记。值得注意的是，只有视频标签可用，数据集中每个轨迹的身份是未知的。关于这两个模拟数据集的更详细信息显示在表I中。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/e582b8c53d134286bce28f20f9b21cf4.png#pic_ center" width="70%" />
</div>

我们还考虑了一个更实际的场景，即袋子可能包含一些噪声轨迹，例如背景图像（轨迹）、同一身份的多个轨迹和没有标签的轨迹。为了模拟这种情况，我们随机向每个弱标记袋子中添加3-6个噪声轨迹。WLMARS*表示在WL-MARS数据集上添加了噪声轨迹的数据集。因此，新的袋子由原始和新添加的人物轨迹组成，但标签与原始相同。

**评估协议**：我们使用累积匹配特征（CMC）曲线，即rank-k准确率，和平均平均精度（mAP）作为评估提出方法的指标。

## B. 实现细节

我们采用了在ImageNet上预训练的ResNet-50模型作为WS-TAL的特征提取器的骨干网络，后面跟有一个2048维的全连接层（d = 2048）作为特征提取层，以及一个C维的分类层来预测身份概率。所有人像图像都被调整为256×128像素作为输入。使用动量为0.9的随机梯度下降来优化我们的模型。学习率初始化为2×10^-4，并在40个周期后变为2×10^-5。训练过程在45个周期后结束。批量大小设置为两个（Nb = 1），这意味着在每次迭代中，我们向模型输入两个袋子。在每个批次中，袋子对必须至少有一个共同的人物身份。在每个训练袋子中，人物图像的数量是一个固定的值72（K = 72）。如果袋子中的图像数量大于这个数字，我们随机从所有轨迹中选择72个人像图像，并用袋子的视频标签标记这个子集。我们使用Pytorch实现我们的方法，所有实验都在一张NVIDIA 2080Ti GPU上进行。我们经验性地将λ1 = 20和λ2 = 20设置在（11）中。提出的WS-TAL模型的具体训练过程可以在算法1中找到。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/db2eb91468b44a8182c3e8f4c00b4525.png#pic_ center" width="70%" />
</div>

## C. 与最先进方法的比较

在这一部分，我们比较了我们的WS-TAL与现有的最先进的人物重识别方法在T2V重识别和T2T重识别任务中的识别性能。这些方法的识别性能显示在表II和表III中。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/212e0ea6990d43d6a6b001456505b526.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/ca241eb939144ab69a59075a083608c2.png#pic_ center" width="70%" />
</div>

**T2V人物重识别**：我们在表II中将我们的方法的识别性能与现有的最先进的弱监督方法HSLR、DeepMIML、MIL-CPAL和DGL进行了比较。这些结果表明，我们的方法明显优于其他方法。在WL-MARS数据集上，HSLR只能达到34.9%的rank-1和22.6%的mAP。与DeepMIML相比，在WL-MARS数据集上，rank-1和mAP分别提高了46.3%和52.4%，同样在WL-DukeV数据集上分别提高了37.0%和37.1%。我们的模型在WLMARS上比MIL-CPAL高出9.5%的rank-1和33.6%的mAP，同样在WL-DukeV上高出7.7%和16.0%。与DGL相比，在WL-MARS数据集上，rank-1和mAP分别提高了5.5%和6.6%。我们的方法在WL-MARS数据集上达到了最佳的T2V重识别rank-1准确率88.1%，在WL-DukeV数据集上达到了90.3%。值得注意的是，为了公平比较，所有四种方法都使用了相同的骨干网络进行评估。我们的方法有更好的性能，主要有两个优势。首先，ITDL可以通过为袋子中的每个人物图像分配伪标签来捕获人物身份和图像之间的关联，以学习每个人的特征。其次，我们在CTAL中不仅挖掘正样本轨迹对，还挖掘相应的硬负样本对，以探索袋子之间的潜在轨迹关联。

**T2T人物重识别**：在表III中，我们将我们的方法与不同标签设置下的现有最先进的T2T人物重识别方法进行了比较，包括无监督方法：BUC、SSL；内摄像头和单样本监督方法：EEA、EUG和VOLTA；全监督方法：DuATM和NVAN；以及弱监督方法：HSLR、DeepMIML、MIL-CPAL和DGL。与SSL（无监督人物重识别）相比，我们的模型在WL-MARS数据集和WL-DukeV数据集上分别实现了21.1%和13.3%的rank-1提高。无监督方法的性能较低，因为缺乏手动标签指导。与单样本监督方法相比，我们的模型在WL-MARS数据集上比VOLTA高出17.2%的rank-1和23.3%的mAP。我们方法性能更好的主要原因是，有视频级标签作为约束来指导伪标签的分配，确保生成的伪标签比单样本重识别生成的更可靠。值得注意的是，我们的方法甚至优于全监督人物重识别模型。与全监督人物重识别相比，我们方法的识别性能与最佳全监督方法NVAN接近（即，在WL-MARS数据集上，rank-1准确率为83.9%对90.0%）。与弱监督人物重识别方法相比，我们的模型在WL-MARS上比MIL-CPAL提高了18.9%的rank-1准确率和29.2%的mAP得分，同样在WL-DukeV数据集上提高了19.2%和22.5%。与DGL相比，在WL-MARS数据集上，rank-1准确率和mAP得分分别提高了7.6%和9.5%。这些结果充分证明了我们方法的有效性。

## D. 超参数分析

在我们的框架中，ITDL和CTAL联合优化以学习提出的WS-TAL模型。在这一部分中，我们研究了两个损失函数对识别性能的相对贡献。为了做到这一点，我们在WL-MARS数据集上进行了不同λ1（控制ITDL的相对重要性）和λ2（控制CTAL的相对重要性）值的实验，并且以热图的形式展示了T2T重识别任务的rank-1准确率和mAP得分。如图4所示，我们通过设置不同的λ1（λ1 = 1, 5, 10, 15, 20, 25）和λ2（λ2 = 0, 1, 5, 10, 15, 20, 25）来评估识别性能。结果表明，我们的模型在两个超参数的一定范围内是稳定的（例如，10 < λ1 < 20, 10 < λ2 < 20）。我们在实验中将λ1设置为20，λ2设置为20，因为在这种情况下，模型可以实现最佳的识别性能83.9%的rank-1准确率和75.2%的mAP得分。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/6e77fbcf444c496984ca8815895eafdf.png#pic_ center" width="70%" />
</div>

## E. 鲁棒性分析

不同噪声轨迹比例下的模型性能：在本节中，我们分析了在不同噪声轨迹比例下提出的模型的性能。为了模拟实际场景，我们随机选择了原始WL-MARS数据集中的一部分（50%，80%，100%）的袋子，并在每个选中的袋子中添加了3-6个噪声轨迹。值得注意的是，WL-MARS*数据集表示100%的袋子都添加了噪声轨迹。在图5中报告了我们的模型在WL-MARS数据集上添加噪声轨迹情况下的识别性能。可以观察到，当向50%的训练袋子添加噪声轨迹时，T2V重识别的rank-1和mAP仅下降了0.2%和4.3%，对于T2T重识别分别下降了1.3%和3.4%。即使在100%的袋子中添加噪声轨迹，T2V重识别的rank-1准确率的性能仅下降了5.1%。因此，我们可以得出结论，提出的方法在这种实际场景下也是可靠的。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/175f73a54b8f4df5ae9cc3dd14b6fabb.png#pic_ center" width="70%" />
</div>

在100%噪声袋子情况下弱监督方法的比较：我们在表IV中将我们的模型与其他弱监督方法在WL-MARS*数据集上进行了T2V和T2T人物重识别的比较。我们的模型即使在100%噪声袋子情况下，仍然能够实现83.0%的rank-1和69.8%的mAP用于T2V重识别，对于T2T重识别，分别达到了75.3%和63.1%。对于T2T重识别，我们的模型在WL-MARS*数据集上比DeepMIML提高了42.9%的rank-1和45.0%的mAP。与MIL-CPAL相比，识别性能分别提高了15.5%和22.5%的rank-1准确率和mAP得分。与DGL相比，我们的模型在T2T重识别方式中分别获得了5.2%和6.0%的rank-1和mAP的提升，而在T2V重识别方式中分别提高了4.4%和2.4%。这些结果充分证明了我们方法在实际使用情况下的有效性。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/287f7ad7b3b34e0f96199495ddf2c637.png#pic_ center" width="70%" />
</div>

不同袋子多样性下的模型性能：我们还评估了我们的模型在不同袋子多样性（即每个袋子中身份的数量）下的鲁棒性。首先，我们假设了一个理想情况，即每个袋子包含固定数量的人物。如表V所示，我们在WL-MARS数据集上评估了我们的方法在T2T重识别方式下，每个袋子分别有3、4、5和6个人物时的性能。我们发现随着袋子中人物数量的增加，识别性能逐渐下降（rank-1准确率下降了3.9%，mAP得分下降了6.1%）。然而，在实际场景中，视频片段中出现的人物数量是相当不确定的。为了更好地模拟这一现实场景，我们随机为每个袋子选择了1-6、3-6、1-8、3-8和1-10个不同人物身份。表V中的结果还表明，我们提出的方法在实际情况下是鲁棒的。与默认设置（每个袋子3-6个轨迹）相比，在不同袋子多样性下，识别性能的变化很小，例如在包括1-8个人物身份时，rank-1和mAP分别仅下降了1.4%和1.9%，当每个袋子有1-10个身份时分别下降了1.4%和2.3%。这些结果表明，我们的方法在各种袋子多样性下都是可行和鲁棒的。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/3e559d4a83174652ba806dc16d491bda.png#pic_ center" width="70%" />
</div>

## F. 伪标签分配和有效轨迹选择过程分析

在本节中，我们深入分析了伪标签的分配准确性以及有效轨迹的选择过程，以评估我们方法的有效性。

伪标签的准确性：我们的方法在ITDL中利用生成的伪标签作为模型优化的监督信号。为了验证这种设置的有效性，我们计算了所选择有效轨迹的伪标签准确性。如图7(a)所示，随着模型训练的进行，伪标签的准确性逐渐提高，并最终达到了98.6%。具体来说，在第三个epoch时，准确性达到了92.0%，并在随后的epoch中保持在90%以上，这表明我们的方法能够在模型训练过程中保证伪标签的质量。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/7ab24e83c7894c61a65a49b02578ec17.png#pic_ center" width="70%" />
</div>

有效轨迹的选择过程：我们的方法在ITDL中选择具有最高身份率的轨迹作为有效轨迹进行模型训练。为了验证我们方法的合理性，我们观察了每个训练epoch中所选择有效轨迹的变化，通过计算变化比率和有效轨迹的比例来实现。

1.**变化比率** 定义为在两个连续训练epoch中选择的不同有效样本的数量与一个训练epoch中选择的总样本数量的比例。从图7(b)中可以看出，随着模型训练的进行，所选择的样本会动态更新，变化比率在83.7%到3.07%之间变化。值得注意的是，即使在模型收敛时，也存在3.07%的变化比率，这意味着模型在每个训练epoch中会选择非常不同的轨迹。

2.**有效轨迹的比例** 定义为所有以前训练epoch中选择的所有有效轨迹的比例与总训练轨迹数的比例。如图7(c)所示，随着模型训练的进行，越来越多的不同有效轨迹会被选择，当模型收敛时，有93.3%的训练样本已经被用于训练过程，表明几乎所有数据都经过了训练。

## G. 在SYSU-30K数据集上的性能

为了进一步验证我们方法的有效性，我们在SYSU-30K数据集上进行了评估。SYSU-30K是一个大规模的基于图像的弱监督重识别基准，包含30,000个个体和29,606,918张图像。为了展示我们方法的有效性，我们将与一些基于迁移学习、自监督学习和弱监督学习的最新方法进行比较。

具体来说，为了适应我们的方法，我们将连续的帧（40-80张图像）连接起来作为轨迹，并随机地将3-6个人轨迹，具有不同身份，组合成一个袋子，构建了弱监督SYSU-30K视频数据集作为训练数据集。表VI中的结果表明，我们的模型在rank-1准确率达到了34.4%，展示了我们方法的优越性。值得注意的是，所比较方法的报告结果来源于[23]和[67]。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/b33599cf44e94eca97c4a4c17721aa41.png#pic_ center" width="70%" />
</div>

## H. 消融研究

为了深入理解我们提出的WS-TAL模型中不同组件的贡献，我们进行了消融研究来评估ITDL（袋内轨迹判别学习）项和CTAL（跨袋轨迹关联学习）项的优势。此外，我们还探讨了过滤过程和硬负样本对挖掘策略的有效性。

ITDL和CTAL的有效性：我们在WL-MARS和WL-MARS*数据集上对T2V重识别任务进行了消融研究。从表VII中可以看出，与DeepMIML相比，提出的ITDL将rank-1准确率从41.8%提高到75.1%，并将mAP得分从28.3%提高到65.0%。此外，引入CTAL项后，识别性能得到了一致性的提升。例如，在WL-MARS数据集上，DeepMIML+CTAL相比于没有CTAL的结果，rank-1准确率和mAP得分分别提高了21.0%和19.2%。结合ITDL和CTAL（即ITDL+CTAL），我们的模型在WL-MARS数据集上达到了88.1%的rank-1准确率和80.7%的mAP，以及在WL-MARS*数据集上的83.0%和69.8%。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/e1486114346c4010bc994132137d6dec.png#pic_ center" width="70%" />
</div>

过滤过程的有效性：如表VII所示，我们评估了ITDL中使用的过滤过程的影响。可以观察到，当过滤过程缺失时，特别是在WL-MARS*数据集上进行T2V重识别时，模型的性能显著下降。具体来说，rank-1准确率从83.0%下降到74.6%，mAP得分从69.8%下降到63.3%。潜在的原因是噪声轨迹的存在使得视频标签不可靠，并增加了有效轨迹的错误率，导致错误的身份特征。实验结果表明，过滤过程可以有效地消除噪声轨迹的负面影响。

硬负样本对挖掘策略的有效性：在本节中，我们评估了CTAL中使用的硬负样本对挖掘策略的效果。如表VIII所示，我们观察到提出的硬负样本对挖掘策略比使用所有负样本对模型训练更有效。引入硬负样本对后，T2V重识别的rank-1准确率和T2T重识别分别提高了3%和0.9%。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/17295b93a08e453b961924a8e27a749c.png#pic_ center" width="70%" />
</div>

## I. 结果可视化

为了更直观地理解T2V和T2T重识别的结果，我们在图6中展示了在WL-MARS数据集上通过我们提出的WS-TAL框架获得的一些结果。图6(a)展示了T2V重识别的结果。可以看到，对于只包含一个轨迹的查询袋子，会返回多个检索到的袋子。在每个袋子中，与查询人物最相似的帧由一个点表示。正确和错误的检索结果分别用绿色和红色点标记。绿色边框表示与查询人物具有相同身份的轨迹。在图6(a)中，我们发现对于袋子3，搜索结果是正确的，但最相似帧是错误的。这可能解释了T2V重识别性能为什么优于T2T重识别。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/d167bc8f57f14ffd901fffd931889798.png#pic_ center" width="70%" />
</div>

对于T2T人物重识别，查询和图库样本都是轨迹，一些检索结果在图6(b)中展示。在T2T重识别中，给定一个查询轨迹，目标是检索图库集中具有相同身份的轨迹。图6(b)中的可视化结果提供了对我们模型在T2T检索任务上性能的直观理解，其中正确和错误的匹配同样用不同颜色进行了区分。

# V. 结论

在本文中，我们提出了一个有效的人物重识别模型WS-TAL，用于处理只有视频标签的弱监督人物重识别任务，而不需要精确地标记视频中的每个轨迹。我们的模型通过联合优化两个互补的项：袋内轨迹判别学习（ITDL）和跨袋轨迹关联学习（CTAL），来训练所提出的模型。在WL-MARS和WL-DukeV数据集上的严格实验表明，我们的方法在T2V和T2T人物重识别任务上取得了最先进的结果。此外，我们还证明了在两种实际场景下提出的方法的有效性和鲁棒性——即袋子中存在噪声轨迹以及每个袋子中轨迹数量的变化情况。

声明
本文内容为论文学习收获分享，受限于知识能力，本文队员问的理解可能存在偏差，最终内容以原论文为准。本文信息旨在传播和学术交流，其内容由作者负责，不代表本号观点。文中作品文字、图片等如涉及内容、版权和其他问题，请及时与我们联系，我们将在第一时间回复并处理。
