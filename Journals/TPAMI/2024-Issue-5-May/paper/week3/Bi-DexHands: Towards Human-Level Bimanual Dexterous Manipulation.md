# 论文题目


# 题目：[Bi-DexHands: Towards Human-Level Bimanual Dexterous Manipulation](https://ieeexplore.ieee.org/document/10343126/)  
## Bi-DexHands: 迈向人类水平的双手灵巧操控
**作者：Yuanpei Chen; Yiran Geng; Fangwei Zhong; Jiaming Ji; Jiechuang Jiang; Zongqing Lu; Hao Dong; Yaodong Yang** 

*****

# 摘要
实现机器人在灵巧操控方面的人类水平仍然是一个关键的开放问题。即使是简单的灵巧操控任务，由于自由度的高数量和异构代理（例如，手指关节）之间合作的需求，也带来了显著的困难。尽管一些研究人员已经利用强化学习（RL）来控制单手操控物体，但需要双手协调合作的任务仍然未被充分探索，因为适合的环境较少，这可能导致困难和次优性能。为了应对这些挑战，我们介绍了Bi-DexHands，这是一个具有两只灵巧手的模拟器，包含20个双手操控任务和数千个目标对象，旨在基于认知科学研究匹配不同水平的人类运动技能。我们在Issac Gym中开发了Bi-DexHands，使用单个NVIDIA RTX 3090实现了超过30,000帧每秒的高效RL训练。基于Bi-DexHands，我们全面评估了不同设置下的流行RL算法，包括单代理/多代理RL、离线RL、多任务RL和元RL。我们的发现表明，策略类算法，如PPO，可以掌握与48个月大婴儿相对应的简单操控任务，例如接住飞行物体或打开瓶子。此外，多代理RL可以提高执行需要熟练双手合作的操控能力，例如举起锅或堆叠积木。尽管在单项任务中取得了成功，但当前的RL算法在大多数多任务和少样本学习场景中难以学习多种操控技能。这突出了RL社区内进一步研究和开发的需求。

# 关键词
- 强化学习 (Reinforcement learning)
- 灵巧操控 (Dexterous manipulation)
- 仿真 (Simulation)

# I. 引言
灵巧操控是机器人与物理世界互动和执行复杂任务的基本技能，例如精确和熟练地抓取、操控和控制物体[1], [2], [3]。它在我们日常生活中的许多活动中都是必需的，从执行如写作或在键盘上打字这样的精细运动任务到处理复杂的工具和设备。发展灵巧操控对机器人学和人机交互至关重要，并且可以为理解复杂运动技能背后的神经和生理机制提供洞见。通过提高机器人手的灵巧性和智能，特别是在双手合作方面，我们可以创造更多能力和多功能的机器人系统，以协助人类在各种领域，如制造业、服务业、娱乐和教育。

双手灵巧操控对机器人来说是一个重大挑战，并且在机器人学中仍然是一个未充分发展的领域[4]。它为两只手之间的合作提供了一个独特的机会和挑战。首先，它需要高维连续控制以确保机器人成功完成任务并精确操控。此外，与单手操控不同，双手操控还需要两手协作来完成涉及两手协调和避免相互干扰的任务[5]。图1(b)展示了一些成功和失败的双手灵巧操控任务的例子，涉及双手灵巧手的合作。这些任务展示了实现所需结果的手部协调和合作的需求，对机器人来说是一个困难的挑战。手部必须协同工作，而不仅仅是考虑它们各自的任务。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/a3e92421a921401880b68391fa11416c.jpeg" width="800" />
</div>





强化学习（RL）算法在诸如手内灵巧操控[1], [2], [3]和抓取[6], [7], [8]等机器人任务中展示了显著的成功，超越了依赖某些假设的传统控制方法。虽然已经实现了像投掷[9]、滑动[10]、戳刺[11]、转动[12]和推动[13]这样的简单操控任务，但将这些方法推广和适应到非结构化和接触丰富的环境仍然是一个挑战。此外，RL处理高维观测，例如原始像素图像或点云，也是困难的，因为机器人需要识别对象并估计手和对象的关节状态[14]。此外，机器人的双手协调挑战由于需要机器人同时学习协调双手的运动而变得更加复杂。当任务涉及协调操控物体时，随着维度的增加、非平稳动力学和非对称合作，挑战变得更加困难。对于从试错互动中学习的RL算法来说，一个高质量的模拟器也是必需的。尽管存在各种机器人操控基准测试，如[15], [16], [17]，但目前缺乏专门为复杂性设计的模拟器双灵巧手操控任务。一言以蔽之，达到手部灵巧性和双手协调的人类水平的复杂性仍然是现代机器人研究人员面临的一个开放挑战。

在本文中，我们构建了一个名为Bi-DexHands的仿真平台，通过RL算法使机器人能够实现人类水平的双手灵巧手操控能力。该平台包括20个任务，遵循Fine Motor Subtest (FMS)[18]的原则，该测试评估儿童使用双手玩耍、操控物体和使用工具的能力。接下来，我们评估了各种无模型RL算法，以展示基线算法在这些任务中的能力，不仅是标准RL算法，还有多代理RL (MARL)、离线RL、多任务RL和元RL算法，每种算法都专注于双手协作、从演示中学习以及任务泛化。我们的主要目标是促进研究人员通过RL掌握人类水平的双手灵巧操控。不仅限于此，我们还希望这项研究为RL社区、机器人学、计算机视觉和认知科学提供一个新的平台。Bi-DexHands具有以下关键特性：
- 高吞吐量：在Isaac Gym[19]仿真器的基础上，Bi-DexHands支持同时运行数千个环境。在单个NVIDIA RTX 3090 GPU上，Bi-DexHands可以通过并行运行2,048个环境达到每秒30,000+的平均帧率。
- 全面的RL基准测试：我们为常见的RL、MARL、离线RL、多任务RL和元RL从业者提供了第一个双手操控任务环境，以及SOTA连续控制无模型RL方法的全面基准测试。
- 异构代理合作：Bi-DexHands中的代理（即关节、手指、手...）是真正的异构的；这与常见的多代理环境如SMAC[20]不同，后者的代理可以简单地共享参数来解决任务。
- 任务泛化：我们引入了多种灵巧操控任务（例如，交接、举起、投掷、放置、放入...）以及来自YCB[21]和SAPIEN[22]数据集的大量目标对象，从而允许元RL和多任务RL算法在任务泛化方面进行测试。
- 感知：我们提出使用多模态信息作为策略观测来进行机器人研究，包括接触力、RGB图像、RGB-D图像、点云。这些观测提供了一套全面的信息，可用于训练机器人执行各种任务。
- 认知：我们提供了我们灵巧任务与不同年龄人类运动技能之间底层关系。这有助于研究人员研究机器人技能学习和发展，特别是与人类的比较。

本文是之前会议论文[23]的扩展，在其中我们增加了新工作以增加其功能性和全面性。具体来说，1)为每个环境提供了额外的多模态视觉观测作为强化学习的输入，包括接触力、语义RGB图像、RGB图像、RGB-D图像和点云。2)对点云作为观测的使用进行了基准测试，并提供了对使用视觉输入的双手操控的分析和讨论。3)启用了使用PPO为每个任务收集演示数据的功能，使用户更容易使用如DAPG[24]这样的教师-学生类方法。4)增加了用户自由切换手部的功能，例如Allegro手，以及将不同的机器人手臂连接到手部，而不仅仅是飞行手。

# III. 公式与算法

为了创建一个平台以掌握人类水平的灵巧性，我们使用两个Shadow Hand在我们的环境进行操控。Shadow Hand是一种流行的机械手，通常用于一些灵巧操控任务。它被设计来模仿典型的成年男性手的形状和大小，并能够执行各种灵活和精细的操作。Shadow Hand的自由度（DoF）如图2所示，旨在尽可能地模仿人类的骨骼结构。具体来说，24-DoF的手由20对激动-拮抗肌腱驱动，而其他四个关节保持欠驱动。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/fe80974d800c475ab39205ded7ef4ad7.jpeg" width="800" />

</div>

我们的低级控制器以1kHz的频率运行，同时基于RL的策略以30Hz的频率输出被驱动关节的相对位置。值得注意的是，与以往的研究相比，某些任务中手的基座不是固定的。相反，策略可以控制基座在受限空间内的位置和方向，这利用了手腕的功能，从而使Shadow Hand更加仿生。同时，我们可以通过将基座连接到机械臂来高效地在现实世界应用中执行任务。有关Shadow Hand物理参数的更多详细信息，请参阅在线提供的附录A.1。

我们的平台旨在为广泛的RL领域提供双手灵巧操控的解决方案。为了实现这一目标，我们考虑了五个RL公式，包括：单代理RL、多代理RL (MARL)、离线RL、多任务RL和元RL在Bi-DexHands中。在接下来的部分中，我们将介绍这五个RL公式的详细公式和我们平台上实现的相应算法。

## RL/MARL

为了评估RL/MARL的性能[54], [55]，我们将我们的场景制定为一个分散的部分可观察MDP（Dec-POMDP）。Dec-POMDP由10个组成部分组成，$Z = \langle N, M, S, O^N, A^N, \Pi^N, P, R, \rho, \gamma \rangle$。最初，机器人手被手动分离为N个代理，其集合表示M。当开始模拟时，环境的状态（即，机器人和对象的信息）根据状态的初始分布 $\rho(s_0)$ 设置为 $s_0 \in S$。然后在时间步 t，$s_t$ 表示状态，第 i 个代理接收到基于 $s_t$ 的观测 $o_t^i \in O^N$。此后，第 i 个代理的策略 $\pi_i \in \Pi^N$ 将 $o_t^i$ 作为输入，并输出动作 $a_t^i \in A_i$。

此外，我们通过 $AAA = [A_1, \ldots, A_N]$ 自然满足方程，表示所有代理的联合动作。之后，第 i 个代理可以根据 $R(s_t, a_t)$ 获得奖励 $r_t^i$，并且所有代理以转移函数 $P(s_{t+1}|s_t, a_t)$ 的可能性转移到下一个状态 $s_{t+1}$。目标是找到最优策略 $\Pi^N$，以最大化一个剧集中 T 个时间步的总奖励期望 $E_{\Pi}^{\Pi^N}[\sum_{t=0}^{T-1} \gamma^t \sum_{i=1}^{N} r_t^i]$。应当指出，当 N = 1 时，它就是单代理RL的问题公式。

在这种设置下，我们实现了最先进的连续单代理RL算法，如PPO[56]、SAC[57]、TRPO[58]、DDPG[59]、TD3[60]和DAPG[24]算法。考虑到我们的连续控制和完全合作环境，我们引入了HAPPO/HATRPO[61]、[62]、[63]、MAPPO[64]、IPPO[65]和MADDPG[66]算法。

## 离线RL

离线RL遵循标准MDP的公式，其中目标是最大化预期回报 $E_{\pi}[{\sum_{t=0}^{T-1} \gamma^t r_t}]$。然而，在离线RL中，代理必须仅使用之前收集的数据集 $D = \{(s_t, a_t, s_{t+1}, r_t)\}$ 来学习策略，而不需要与环境互动。离线RL的基本挑战是分布外动作的价值误差[67]。我们为离线RL实现了BCQ[68]、TD3+BC[67]和IQL[69]算法。

## 多任务RL

多任务强化学习的目标是训练一个单一的策略 $\pi(a|s, z)$，在不同任务上都能取得良好的结果。z 表示任务ID的编码。我们策略的目标是最大化奖励 $E_{T \sim p(T)}[E_{\pi}[{\sum_{t=0}^{T-1} \gamma^t r_t}]]$，其中 p(T) 是我们平台上的任务分布。在实践中，多任务RL通过将与环境类型对应的上下文向量（例如，一位任务ID）添加到状态中来学习一般技能。我们为多任务RL实现了多任务PPO、多任务TRPO和多任务SAC算法。

## 元RL

元RL[70]，也称为“学会学习”，旨在获得在任务上训练的能力，以提取这些任务的共同特征，从而快速适应新的和未见过的任务。在元RL中，训练和测试环境都被假设遵循相同的任务分布 p(T)。在BiDexHands中，我们为元训练设计了一些不同任务之间的共同结构，以确保它可以有效地适应新任务。与多任务RL相比，元RL不允许直接获取任务级信息，如任务ID。它需要通过基于互动的任务推断和适应来解决完全新的任务。我们为元RL实现了模型无关元学习（MAML）[71]和近端元策略搜索（ProMP）[72]算法。

# IV. 全面的Bi-DexHands平台

在本节中，我们将讨论Bi-DexHands的构建，这是一个用于多种场景下双手灵巧操控的仿真平台。

## A. 系统设计

正如我们之前提到的，Bi-DexHands的核心是构建一个学习框架，使两个Shadow Hand能够像人类一样掌握各种技能，例如伸手、投掷、接住、捡起和放置。具体来说，Bi-DexHands由三个组成部分构成：数据集、任务和学习算法，如图3所示。不同的世界为机器人提供了大量的基本设置，包括机械手和对象的配置。同时，对应不同年龄段儿童行为的各种任务使得学习像人类一样的灵巧操控成为可能。结合数据集和任务，我们可以为以下学习生成特定的环境或场景。最终，我们的实验表明，强化学习能够促进机器人在这些具有挑战性的任务上取得一些显著的性能，并且对于未来的工作还有改进的空间和更困难的任务。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/a68f2a65df414dc69ab75919eb43de7c.jpeg" width="800" />

</div>

## B. 数据集的构建

数据集的构建对应于机器人和对象的配置。数据集的核心目标是为机器人学习生成大量不同的场景。正如我们在上一部分提到的，我们平台上的机器人是两个灵巧的Shadow Hand。除了机器人，对象在构建数据集中也起着重要作用。为了扩展任务类型，我们从YCB[21]和SAPIEN[22]数据集中引入了各种对象。这两个数据集包含许多日常对象。值得注意的是，SAPIEN数据集提供了许多具有运动注释和渲染材料的可动物体，这意味着这些对象与真实对象非常接近。因此，它为我们提供了一种自然的方式来建立我们平台世界与日常生活场景之间的联系。具体来说，图3显示了数据集的构建，我们可以看到对象包括锅、笔、鸡蛋、剪刀、眼镜、门和其他常见工具。在定义了机器人的配置和对象类型之后，我们基于Isaac Gym仿真器构建了特定的世界。同时，每个世界定义了机器人和对象的可变初始姿态，提供了一组多样化的环境。

## C. 任务设计

婴儿的行为经历了多阶段的发展，例如社交、沟通和身体部分[73]。特别是在双手灵巧操控方面，一些常见的婴儿行为与年龄有关。为了深入了解这些底层关系，我们进行了深入分析，并根据Fine Motor Subtest (FMS)[18]建立了婴儿年龄和任务之间的映射。随着婴儿年龄的增长，完成任务的难度也在增加，因为随着身体的发展，婴儿能够完成越来越多和越来越困难的行为。因此，评估训练代理的性能也非常重要，因为通过类比，我们可以大致指出代理的智能水平，以用于双手灵巧操控。我们的任务与FMS对应的概览显示在表II中。有关任务的更多详细信息，请参阅在线提供的附录A.2。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/abbd034e3ad14ac3a2fc8af72e98fa07.jpeg" width="800" />

</div>
<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/66b3779a086145ee8a2c858176bf8ffe.jpeg" width="800" />

</div>

## D. 多任务/元RL设计

我们的多任务/元RL类别的设计通常与Meta-World[15]相似，分为ML1、MT1、ML4、MT4、ML20和MT20。我们设计的每个任务都具有对象变化，这为我们在日常生活中与不同类型的对象互动提供了基础，为我们学习像人类一样的灵巧操控提供了基础。在以下部分，我们将介绍6个多任务/元RL类别的任务。图4可视化了我们多任务和元类别的详细设计。更多详细信息，请参阅在线提供的附录D。


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/9040fd92622644c0b800fe79be251940.jpeg" width="800" />

</div>


MT1 & ML1: 在同一任务内学习多任务策略和少样本适应。ML1和MT1都是为了反映同一任务内泛化能力的类别，它们的泛化能力体现在在不同目标下完成任务的能力上。ML1使用元强化学习进行少样本适应，其中目标信息将不会被提供。MT1使用多任务方法进行泛化，目标信息将在固定集中提供。

MT4 & MT20: 学习属于4到20个训练任务的多任务策略。MT4和MT20在4到20个任务中进行策略训练，并希望仅使用一个策略完成所有任务。在MT4中，我们希望学习与人类技能类似的策略，因此我们尽可能多地结合相似的任务。MT20使用了我们所有的20个任务。在MT4和MT20中，我们使用一位任务ID来表示不同的任务，目标信息将在固定集中提供。

ML4 & ML20: 从3到15个训练任务中学习新1到5个测试任务的少样本适应。ML4和ML20分别是在3到15个任务中学习元策略的类别，并希望适应新的1到5个测试任务。毫无疑问，这是一个困难的挑战。我们在ML4中选择使用捕捉行为设计的任务。ML20要求在根据婴儿智能设计的具有巨大差异的所有15个任务中进行适应，这是我们平台上最困难的挑战。同样，我们将为每个任务变化目标，并不会提供任务信息，要求元RL算法识别任务。

## E. 离线数据收集

我们遵循D4RL[30] mujoco任务的数据收集。中等数据集是通过首先使用PPO在线训练策略，提前停止训练，并使用该中等策略收集$10^6$个样本($s_t, a_t, s_{t+1}, r_t)$生成的。随机数据集由随机初始化的策略收集，包含$10^6$个样本。回放数据集由中等策略训练期间的$10^6$个经验样本组成。中等-专家数据集由专家策略和中等策略收集的样本等量混合而成，包含$2 \times 10^6$个样本。为了跨任务进行比较，按照D4RL[30]的设置，我们将每个任务的分数标准化到0和100的范围内，通过计算标准化分数 = $\frac{100 \times \text{return} - \text{return}_{\text{random}}}{\text{return}_{\text{expert}} - \text{return}_{\text{random}}}$。

标准化分数为0对应于代理在动作空间中随机采取行动的平均回报。分数为100对应于专家策略的平均回报。

## F. 视觉观测设计

视觉观测对于强化学习（RL）在现实世界场景中的应用至关重要，因为在现实世界场景中，直接访问对象的状态可能不可行。Bi-DexHands提供了几种选择，使用RGB图像、RGBD图像和点云作为观测来训练RL策略。在这种情况下，使用一个或多个相机捕获RGB和RGBD图像，然后将这些图像转换为点云。在常规设置中，我们在手附近放置了一个RGBD相机。为了确保手和对象的最佳视图，我们为每个任务精心设计了相机的位置和焦点。在视觉观测的情况下，教师-学生方法是流行的方法。这类方法使用易于观测（例如6D姿态）训练的策略作为教师，收集使用困难观测（例如点云）的演示，并使用模仿学习来训练基于演示的学生策略。Bi-DexHands也支持这种方法。对于我们的实验，我们使用使用完整状态信息训练的策略作为教师，并使用所有任务的视觉观测收集了1,638,400步演示来训练学生策略。

总的来说，Bi-DexHands为研究强化学习问题提供了一个多功能的环境，特别是在需要视觉观测的设置中。凭借对各种观测类型和相机设置的支持，它使研究人员能够训练能够在非结构化和接触丰富的环境中执行复杂操控任务的RL策略。

## G. 可定制的多样化操控器

除了像Allegro手、三指手等灵巧手之外，支持其他灵巧手有助于推进研究和社区发展。因此，除了Shadow Hand，我们还提供了Bi-DexHands中的四种其他灵巧多指手。此外，使用机器人臂驱动手部的基座不仅符合真实世界的设置，也是模拟到真实转移的一步。因为很难匹配飞行手的真实动态。

此外，我们提供了多种臂和多种灵巧手的组合，这有许多好处。例如，研究人员可以根据他们自己的条件选择他们想要的手，这为我们的平台带来了更广泛的适用性。同时，我们可以使用不同的臂和不同的手来研究策略的适应性和泛化能力，这在未来对多任务学习和元学习研究提出了挑战。这一特性的示意图显示在图5中。


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/91810f8d2f334ce1932eb8e2146f7e33.jpeg" width="800" />

</div>

# V. 实验

## A. 环境设置

在本节中，我们提供了Bi-DexHands中任务组件的详细描述。Bi-DexHands专注于应用RL算法于机械手控制，这在传统控制算法中是具有挑战性的。Bi-DexHands的难度不仅在于复杂的任务内容，还在于高维连续空间控制。每个环境的状态空间维度高达400维，动作空间维度高达40维。Bi-DexHands的一个独特特点是多代理方面，每个手由五个手指和一个手掌组成，可以作为一个最小代理单位使用。这允许将每个手指和手掌作为代理使用，或者根据需要将它们中的任意数量组合为代理。所有环境都是目标导向的，每个时期随机重置对象的起始姿态和目标姿态以提高泛化能力。此外，我们还提供了YCB和Sapien数据集中的对象作为扩展，允许用户自定义他们实验中使用的对象类型。

观测：观测空间由三个部分组成：$(O_{\text{左手}}, O_{\text{右手}}, O_{\text{任务}})$，代表左右Shadow Hand的状态信息和与任务相关的信息。这些任务的观测空间维度范围从414到446。有关每个任务观测空间的更多信息，请参见在线提供的附录A.2。

动作：每个Shadow Hand都有五个手指，其中至少有24个驱动单元，包括四个欠驱动的指尖单元。有20个主动驱动单元，因此每个Shadow Hand的动作空间为20维。双Shadow Hands的动作空间为40维，$A_{\text{左手}} = A_{\text{右手}} = 20$。此外，在大多数任务中（例如，Switch任务），每个Shadow Hand的基座也是可以移动的。这为每个手基座在世界坐标系中的平移和旋转提供了另外6个自由度。每个任务的动作空间的详细信息请参见在线提供的附录A.2。

奖励：我们设计了一些辅助奖励以帮助RL代理更一致地学习，每个任务都包含特定于任务的奖金。通常，我们的奖励设计是目标导向的，并遵循相同的逻辑。对于对象捕获任务，我们的奖励简单地与对象的姿态和目标姿态之间的差异有关。对于其他需要手部持有对象的任务，我们的奖励通常由三部分组成：左手到对象上左手抓握点的距离，右手到对象上右手抓握点的距离，以及对象到其目标的距离。

多代理：在Bi-DexHands的多代理设置中，每个代理的观测部分依赖于它所属手的观测。例如，如果左手远端手指、左手拇指和右手远端手指被视为单独的代理，那么左手远端手指和左手拇指的观测包括整个左手以及对象和目标信息的观测。另一方面，右手远端手指的观测只包括整个右手和对象及目标信息的观测。在Bi-DexHands中，所有代理接收相同的奖励，在完全合作的游戏中进行。

基线：我们提供了我们评估的RL算法的简要介绍，我们在Github仓库中实现了其余的算法。对于单代理RL，我们有PPO[56]、SAC[57]、TRPO[58]和DAPG[24]。对于离线RL，我们有BCQ[68]、TD3+BC[67]和IQL[69]。对于多代理RL，我们有HAPPO/HATRPO[61]、[62]、[63]和MAPPO[64]。对于多任务RL，我们有Multi-Task PPO[56]。对于元RL，我们有近端元策略搜索（ProMP）[72]。更多详细描述可以在在线提供的附录B中找到。

## B. 环境速度

得益于Isaac Gym的高性能GPU并行模拟能力，我们可以在使用较少计算资源的同时大幅提高我们RL算法的采样效率。我们认为高采样效率提高了RL算法的探索能力，使我们能够成功地学习双手灵巧操控策略。为了展示Isaac Gym在Bi-DexHands中的效率，我们在表III中提供了一些环境速度的结果，通过运行策略类算法得出。PPO和HAPPO都可以实现超过20k FPS的速度。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/5d3596e0633d48488d42e88f61310b27.jpeg" width="800" />

</div>

## C. RL/MARL 结果

目前，我们在20个任务上评估了PPO、SAC、TRPO、MAPPO、HATRPO和HAPPO算法的性能。图6显示了每个算法的性能。请注意，MARL算法的实验是基于两个代理运行的，这意味着每只手代表一个代理。可以观察到，PPO算法在大多数任务上表现良好，表明PPO算法在高并行模拟中表现出色，能够完成具有挑战性的任务。尽管有些任务需要双手合作，但在大多数情况下，PPO算法仍然优于HAPPO和MAPPO算法。这可能是因为PPO算法能够使用所有观测来训练策略，而MARL只能使用部分观测。然而，在大多数需要双手合作且更困难的任务中，PPO和HAPPO、MAPPO之间的性能差距较小，表明多代理算法可以提高双手合作操控的性能。


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/677430a8497b457e88e63827d0025641.jpeg" width="800" />

</div>


为了更好地指示每个任务的解决水平，我们还定义了每个任务的成功标准，并使用PPO算法进行了测试。结果如表VI所示。它通常与任务难度对应的婴儿年龄相匹配。年龄越高，任务越困难，成功率越低。这表明基于认知科学文献设计的任务是合理的，从而激励研究人员尝试更多这方面的工作。


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/5f31990a333649b194ada1cf932921d4.jpeg" width="800" />

</div>

另一个发现是SAC算法在几乎所有任务上都不奏效。这可能是由于1) 高采样效率下，离线算法的改进低于在线算法。2) SAC的策略熵在高维输入下为策略学习带来不稳定性。我们在在线提供的附录C中有更详细的讨论。

## D. 离线RL结果

我们构建了四种数据类型的离线数据集，即随机、回放、中等和中等-专家。数据收集遵循D4RL-MuJoCo[30]的标准离线基准测试，详情见第IV-E节。我们在两个任务上评估了行为克隆（BC）、BCQ[68]、TD3+BC[67]和IQL[69]，并在表IV中报告了标准化分数。BCQ和TD3+BC与行为策略（BC）相比，可以获得显著的性能提升。然而，BiDexHands的动作空间和状态空间比MuJoCo大得多，这意味着在Bi-DexHands数据集中，分布外动作的问题比MuJoCo更严重。这就是为什么IQL只能在几个数据集上实现性能提升的原因。由于潜在的大规模分布转移，我们认为Bi-DexHands可以成为一个更具挑战性和意义的离线RL研究基准。


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/9788e7f5638544d581e6889dc0308f47.jpeg" width="800" />

</div>

## E. 泛化能力

我们泛化评估的目标是1) 了解当前多任务和元强化学习算法在我们设计的任务上的泛化能力。2) 了解对婴儿来说更难的任务是否对RL来说也更难。之前的RL/MARL结果已经证明了我们单独的任务是可解的。对于目标1)，我们在MT1、ML1、MT4、ML4、MT20和ML20上评估了多任务PPO[56]和ProMP[72]算法。我们还提供了随机策略和使用PPO算法在单独任务上的结果作为基准进行比较。每个训练的平均奖励如表V所示。我们可以观察到，多任务PPO的表现不佳，而ProMP与随机策略相比只有微小的性能提升。这可能是因为在Bi-DexHands中为每个任务单独学习策略本质上是具有挑战性的。因此，我们仍有很大的改进空间，以提高双手灵巧操控在跨任务设置下的泛化能力，这是社区一个有意义的开放挑战。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/0cddc378c48747239795c95ce78d1f63.jpeg" width="800" />

</div>

对于目标2)，我们使用随机和真实奖励来标准化MT20下所有任务的结果，并按照年龄增加的顺序进行排列。结果如图7所示。可以看出，通常情况下，随着对应任务的人的年龄增加，RL的难度也在增加，这证明了我们的任务设计是合理和与人相关的。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/ee6fcbae7d8343a69a4fe6042d996603.jpeg" width="800" />

</div>


## F. 视觉观测结果

我们对使用视觉观测而非对象状态信息来完成具有挑战性的双手灵巧操控任务的可行性进行了研究。由于视觉观测实验的计算需求较高，我们将实验限制为单摄像头点云观测。我们使用PointNet[74]提取128维的点云特征，该特征与内感受观测串联。视觉输入策略使用PPO算法进行训练。对于教师-学生方法，我们采用DAPG算法在提供的演示上进行训练。我们还提供了使用对象状态信息作为基线的PPO结果。结果如下所示：

在图中，PPO+PC表示使用点云输入训练的PPO算法的结果。观察到使用PC输入的PPO性能较差，特别是在依赖对象信息的投掷和捕获任务中（例如腋下捕获、正面捕获）。这表明直接使用点云代替对象状态可能对任务具有挑战性。然而，DAPG表现良好，大多数任务达到了原始PPO的性能。这表明教师-学生方法是有效的，且DAPG的方差远小于原始PPO。结合使用PC输入的PPO性能不佳，很可能DAPG策略只是记住了灵巧手完成任务所需的动作，并且较少使用视觉信息。因此，提高RL算法利用视觉观测信息的能力可能是一个有前景的研究方向。

# VI. 结论

我们介绍了一个仿真平台Bi-DexHands，它由精心设计的任务和大量对象、灵巧手和臂组成，用于学习双手灵巧操控。我们研究了婴儿灵巧性发育的认知科学过程，并基于此结果为RL设计了二十多种任务，希望机器人能够像人类一样学习灵巧性。借助Isaac Gym仿真器的帮助，它可以并行运行数千个环境，提高了RL算法的样本效率。此外，实现的RL/MARL/离线RL算法在需要简单操控技能的任务上表现良好。同时，复杂的操控动作仍然具有挑战性。特别是，当代理被训练以掌握多种操控技能时，多任务/元RL的结果并不令人满意。另一个挑战是视觉输入RL中视觉信息的有限利用，这是灵巧操控任务使用视觉观测的一个有前景的研究方向。有趣的是，我们发现在多任务设置下，RL表现出与人类智能发展相关联的结果，即RL性能的趋势与人类年龄的发展相匹配。到目前为止，在双手灵巧机器人手操控方面，当前的强化学习可以达到48个月大婴儿的水平。

然而，我们认为Bi-DexHands的局限性在于它不支持可变形物体操控任务。灵巧手在操纵可变形物体方面具有独特优势，但我们的任务目前只涵盖了可关节的刚体物体操控。我们希望在未来能够朝这个方向发展。我们确定了掌握人类水平双手灵巧操控的四个主要未来方向。1) 从演示中学习：我们的平台需要一些人类教学数据来研究从演示中学习。2) 软体和可变形物体仿真：我们需要更好的物理引擎来支持我们的研究，包括软件和任务设计，以更具体地针对日常生活场景。3) 当前的元/多任务RL算法无法成功执行我们平台上的所有任务，这要求在算法设计方面进行大量进一步的开发。4) 我们希望解决仿真到现实的转换问题，通过将仿真结果转移到真实的灵巧手上。特别是，我们希望我们的平台结果可以作为一个起点，帮助研究人员将RL学到的技能转移到现实中，并帮助现实世界的机器人学习灵巧操控。

# 声明
本文内容为论文学习收获分享，受限于知识能力，本文对原文的理解可能存在偏差，最终内容以原论文为准备。  
本文信息旨在传播和交流学术，其内容由作者负责，不代表本号观点。文中内容如涉及作品文字。图片等内容、版权和其他问题，请及时与我们联系，我们将在第一时间删文处理。