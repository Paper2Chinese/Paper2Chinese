# 题目：[Model-Based Reinforcement Learning With Isolated Imaginations](https://ieeexplore.ieee.org/document/10328687/)  
## 基于模型的强化学习与独立想象力
**作者：Minting Pan; Xiangming Zhu; Yitao Zheng; Yunbo Wang; Xiaokang Yang** 

****

# 摘要

在基于视觉的交互系统中，世界模型学习行动的后果。然而，在实际场景中，如自动驾驶，存在不可控制的动态，这些动态独立于或与行动信号稀疏相关，这使得学习有效的世界模型变得具有挑战性。为了解决这个问题，我们提出了Iso-Dream++，这是一种基于模型的强化学习方法，具有两个主要贡献。首先，我们优化了逆动力学，鼓励世界模型从环境混合的时空变化中隔离出可控制的状态转换。其次，我们基于解耦的潜在想象进行策略优化，我们将不可控制的状态滚动到未来，并将其与当前可控制的状态自适应地关联起来。这使得长期的视动控制任务能够从野外隔离混合动态源中受益，例如，能够预测其他车辆运动的自动驾驶汽车，从而避免潜在风险。在我们之前的工作（Pan等人，2022年）的基础上，我们进一步考虑了可控制和不可控制状态之间的稀疏依赖性，解决了状态解耦的训练崩溃问题，并在迁移学习设置中验证了我们的方法。我们的实证研究表明，Iso-Dream++在CARLA和DeepMind Control上显著优于现有的强化学习模型。

# 关键词
- 解耦动力学
- 基于模型的强化学习
- 世界模型。



# I. 引言
人类通过观察和与环境的互动来推断和预测现实世界的动态。受此启发，许多尖端的AI代理使用自监督学习[2]、[3]、[4]或强化学习[5]、[6]、[7]技术来从周围环境中获取知识。在这些方法中，世界模型[3]在机器人视觉运动控制领域受到了广泛关注，并推动了基于模型的强化学习（MBRL）的最新进展。
本文提出了一种理解世界的方法，即通过将世界分解为可控和非可控状态转换，也就是 $s_ {t+1} \sim p(·| s_ t, a_ t)$ 和 $z_ {t+1} \sim p(·| z_ t)$，来根据对动作信号的响应进行区分。这一思想主要受到实际场景如自动驾驶的启发，在这些场景中，我们可以自然地将系统中的时空动态划分为对动作（例如加速和转向）有完美响应的可控部分和代理无法控制的部分（例如其他车辆的运动）。通过这种方式解耦潜在状态转换，可以在以下三个方面提升MBRL的性能：
1. 它允许基于对未来非可控动态的预测来做出决策，这些动态与动作独立（或间接依赖），从而提高长期控制任务的性能。例如，在CARLA自动驾驶环境中，通过预测其他车辆的运动，可以更好地避免潜在风险。
2. 模块化的世界模型提高了RL代理在嘈杂环境中的鲁棒性，正如我们在带有时间变化背景的修改版DeepMind Control Suite中所展示的那样。
3. 进一步隔离可控状态转换有助于不同但相关领域之间的迁移学习。我们可以根据对领域差距的先验知识，将世界模型的部分适应到新的领域。
具体来说，我们提出了Iso-Dream++，这是一个新颖的MBRL框架，它学习解耦并利用可控和非可控状态转换。相应地，它从两个角度改进了原始的Dreamer[6]：
   i. 提出了一种新形式的世界模型表示。
   ii. 提出了一种新的演员-评论家算法，用于从世界模型中派生行为。
## A. 如何学习解耦的世界模型？
从表示学习的角度来看，我们改进了世界模型，将其混合的视觉动态分离为动作条件分支和无动作分支的潜在状态转换（见图1）。这些组件共同训练以最大化变分下界。此外，动作条件分支特别通过逆动态作为额外的目标函数进行优化，即推理出驱动“可控”状态转换的动作。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/e40434372bac4a0d99f1e8fe5eaed32d.jpeg" width="800" />
</div>




然而，正如我们在2022年NeurIPS的先前工作[1]中观察到的，称为Iso-Dream的方法中，逆动态的学习过程容易受到“训练崩溃”问题的影响，即动作条件分支捕获了所有动态信息，而无动作分支几乎什么也没学到。为了防止所有信息都崩溃到单一的状态转换分支中，我们引入了新的最小-最大方差约束来规范解耦世界模型中的动态信息流。更具体地说，我们为世界模型提供一批假设动作，并鼓励动作条件分支基于相同状态产生不同的状态转换，同时对无动作分支中的多样性进行惩罚。



## B. 如何基于解耦的世界模型改进行为学习？
人类可以通过简单的观察和与环境的互动来推断和预测现实世界的动态。受到这一启发，许多尖端的人工智能代理使用自监督学习或强化学习技术来从周围环境中获取知识。在这些方法中，世界模型在机器人视觉运动控制领域受到了广泛关注，并推动了基于模型的强化学习（MBRL）的最近进展。
在本文中，我们提出了一种通过将世界分解为可控和非可控状态转换来理解世界的方法，即 $st+1 \sim p(\cdot | st, at)$ 和 $zt+1 \sim p(\cdot | zt)$，根据对动作信号的响应。这一思想主要受到如自动驾驶等实际场景的启发，在这些场景中，我们可以自然地将系统中的时空动态划分为对动作（例如加速和转向）完美响应的可控部分和代理无法控制的部分（例如其他车辆的运动）。以这种方式解耦潜在状态转换可以在以下三个方面改进MBRL：
1. 它允许基于对未来非可控动态的预测做出决策，这些动态与动作独立（或间接依赖），从而提高长期控制任务的性能。例如，在CARLA自动驾驶环境中，通过预测其他车辆的运动，可以更好地避免潜在风险。
2. 模块化的世界模型提高了RL代理在嘈杂环境中的鲁棒性，正如我们在具有时变背景的修改后的DeepMind Control Suite中所展示的那样。
3. 进一步隔离可控状态转换还促进了不同但相关领域之间的迁移学习。我们可以根据对领域差距的先验知识，将世界模型的部分适应到新的领域。
具体来说，我们提出了Iso-Dream++，这是一种新颖的MBRL框架，它学习解耦并利用可控和非可控状态转换。相应地，它从两个角度改进了原始的Dreamer：
   - 一种新形式的世界模型表示；
   - 一种新的演员-评论家算法，用于从世界模型中派生行为。

# III. 方法
在本节中，我们介绍了Iso-Dream++的技术细节，该方法用于视觉MBRL中可控和非可控动态的解耦和利用。整体流程基于Dreamer，我们从过去的经验数据集中学习世界模型，从想象的紧凑模型状态序列中学习行为，并在环境中执行行为策略以增长经验数据集。
## A. 具有动态隔离的世界模型
受到先前研究[15]、[16]的启发，该研究表明模块化结构对于解耦学习是有效的，我们使用具有多个分支的架构来独立地对不同动态进行建模，根据它们各自的物理定律。每个单独的分支倾向于呈现出即使在其他分支的动态模式发生变化时也保持稳健的特征。具体来说，我们的三分支模型，如图3左侧所示，将视觉观测解耦为可控动态状态 $st$、非可控动态状态 $zt$ 和环境的时间不变组成部分。动作条件分支模拟可控状态转换 $p(st+1 | st, at)$。它遵循PlaNet[11]中的RSSM架构，使用循环神经网络GRUs(·)、确定性隐藏状态 $ht$ 和随机状态 $st$ 来形成转换模型，其中GRU保持可控动态的历史信息。动作自由分支模拟具有类似网络结构的 $p(zt+1 | zt)$。具有独立参数的转换模型可以写为：

$$
p(\tilde{s}_ t | s_ {t:c}, a_ {t:c}) = p(\tilde{s}_ t | h_ t),
$$

$$
p(\tilde{z}_ t | z_ {t:c}) = p(\tilde{z}_ t | h'_ t),
$$

其中 $ht = GRUs(ht−1, st−1, at−1)$, $h'_ t = GRUz(h'_ {t-1}, zt−1)$。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/2d3f3b0af0bc4819962ec4cc9e5c029a.jpeg" width="800" />
</div>




这里我们使用 $\tilde{s}_ t$ 和 $\tilde{z}_ t$ 来表示先验表示。我们通过从 $st \sim q(st | ht, ot, at−1)$ 和 $zt \sim q(zt | h'_ t, ot, at−1)$ 派生出的后验表示来优化转换模型。我们通过共享编码器Encθ和随后的分支特定编码器Encφ1和Encφ2从当前时间步的观测 $ot \in R^{3 \times H \times W}$ 学习后验。值得注意的是，我们将动作输入到Encφ1和Encφ2中，这与我们之前的工作[1]不同。在静态分支中，由于没有状态转换，我们只使用一个编码器Encφ3和一个解码器Decϕ3来模拟环境中的简单时间不变信息。
1) 逆动力学：为了实现与控制信号相对应的解耦表示学习，我们引入了逆动力学的训练目标。这个目标鼓励动作条件分支学习基于特定动作的更确定性的状态转换，而动作自由分支学习与控制信号无关的剩余非可控动态。相应地，我们设计了一个逆单元，一个2层MLP，来推断导致可控状态转换的某些转换的动作：

$$
\tilde{a}_ {t-1} = MLP(st−1, st),
$$

其中输入是动作条件分支中的后验表示。通过学习回归真实行为 $a_ {t-1}$，逆单元促进动作条件分支隔离可控动态的表示。我们分别使用先验状态 $\tilde{s}_ t$ 和后验状态 $\tilde{z}_ t$ 生成可控视觉组件 $\hat{o}_ s \in R^{3 \times H \times W}$ 与掩码 $M_ s \in R^{1 \times H \times W}$ 和非可控组件 $\hat{o}_ z \in R^{3 \times H \times W}$ 与 $M_ z \in R^{1 \times H \times W}$。通过进一步整合从第一K帧提取的静态信息，我们有：

$$
\hat{o}_ t = M_ s \odot \hat{o}_ s + M_ z \odot \hat{o}_ z + (1 - M_ s - M_ z) \odot \hat{o}_ b,
$$

其中 $\hat{o}_ b = Decϕ3(Encθ,φ3(o1:K))$。


对于奖励建模，我们有关动作自由分支的两种选择。首先，我们可以将非可控动态视为与任务无关的噪声，因此不涉及 $zt$ 在想象中。换句话说，策略和预测的奖励将仅依赖于可控状态，例如， $p(r_ t | s_ t)$。或者在其他情况下，我们需要考虑未来非可控状态对代理决策过程的影响，并在行为学习期间纳入动作自由组件。为了实现这一点，我们训练奖励预测器以模拟形式的MLPs  $p(r_ t | s_ t, z_ t)$。


对于从重放缓冲区采样的训练序列 $(o_ t, a_ t, r_ t)_ {T}^{t=1}$，可以使用以下损失函数来优化世界模型，其中α、β1和β2是超参数：


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/984046a6cebf4a708150f252036a9676.png" width="300" />
</div>



### 2) 训练崩溃和最小-最大方差约束
尽管对逆动力学进行了建模，但对于原始的Iso-Dream，我们发现世界模型隔离可控和非可控动态仍然具有挑战性。在初步实验中，我们观察到世界模型训练的解耦结果不稳定，动作条件分支偶尔学习到非可控状态转换的不匹配表示。这意味着大部分有用信息可能崩溃到动作条件分支，而动作自由分支几乎什么也学不到，我们称之为“训练崩溃”。这种现象的出现是由于逆动力学训练目标的固有局限性，可能无法确保完全排除与动作独立的州转换，特别是当动作条件网络分支对建模动态具有很强的能力时。
为了防止状态转换分支发生训练崩溃，我们提出了最小-最大方差约束，其关键思想是（i）在给定不同动作输入的情况下最大化动作条件分支的结果多样性（ii）在相似条件下最小化动作自由分支的结果多样性。为此，与原始的Iso-Dream不同，我们在世界模型学习过程中使动作自由分支也意识到动作信号。但在行为学习和策略部署中，我们将输入动作设置为0值。
计算方差的信息论解释如下：为了研究动态和动作信号之间的联系，世界模型被迫识别提供关于我们对动作信号的信念的信息动态。预期的信息增益可以表示为状态和动作的条件熵：

$$
I(s_ t; a_ {t-1}|s_ {t-1}) = H(s_ t|s_ {t-1}) - H(s_ t|s_ {t-1}, a_ {t-1}).
$$

对于动作条件分支，我们最大化状态和动作信号之间的互信息，以关注特定动作的状态转换。给定一批假设动作 $\{a_ {i,t-1}|i \in [1, n]\}$，对于相同的可控状态 $s_ {t-1}$，我们有基于这些动作的不同状态转换： $\tilde{s}_ {i,t} \sim p(\tilde{s}_ {i,t}|s_ {t-1}, a_ {i,t-1})$， $i \in [1, n]$。经验方差用于近似信息增益，目标可以写为：

$$
L_ s = \max \sum_ {T} Var(\tilde{s}_ {i,t}) = \max \sum_ {T} \left( \frac{1}{n - 1} \sum_ {i} (\tilde{s}_ {i,t} - \bar{s}_ t)^2 \right),
$$

其中 $\bar{s}_ t = \frac{1}{n} \sum_ {i} \tilde{s}_ {i,t}$， $i \in [1, n]$。相反，在动作自由分支中，我们最小化由不同动作导致输出状态的方差，惩罚状态转换的多样性：

$$
L_ z = \min \sum_ {T} Var(\tilde{z}_ {i,t}) = \min \sum_ {T} \left( \frac{1}{n - 1} \sum_ {i} (\tilde{z}_ {i,t} - \bar{z}_ t)^2 \right),
$$

其中 $\bar{z}_ t = \frac{1}{n} \sum_ {i} \tilde{z}_ {i,t}$， $i \in [1, n]$。整体训练目标为：

$$
L_ {all} = L_ {base} + L_ {var},
$$

其中 $L_ {var} = \lambda_ 1 L_ s + \lambda_ 2 L_ z$， $\lambda_ 1$ 和 $\lambda_ 2$ 是超参数。方便起见，我们仅在动作条件分支中使用两个相反的动作 $\{a_ t, -a_ t\}$，在动作自由分支中使用动作集 $\{a_ t, 0, -a_ t\}$ 来计算 $L_ s$ 和 $L_ z$ 。对于后续学习，我们在动作条件和动作自由分支中分别使用 $a_ t$ 和0作为输入动作。

### 3) 解耦状态之间的稀疏依赖
在某些情况下，可控和非可控动态并不完全独立，如图2所示。这在自动驾驶中尤其正确，其中自车的动作可以影响其他车辆的行为，导致它们转向或减速。为了基于当前可控状态准确预测未来的非可控状态，考虑这些稀疏依赖是至关重要的。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/4f24d146994e408398fa53575acaf727.jpeg" width="800" />
</div>



为了有效建模稀疏依赖，关键是要识别可控状态对非可控状态产生显著影响的时刻。为此，我们提出了一个紧凑的模块，称为依赖门，它连接了之前隔离的动作自由和动作条件分支，如图3（左）所示。我们在时间上展开了依赖门的详细结构（见图5），其中可控状态 $\tilde{s}_ t$ 和非可控状态 $\tilde{z}_ t$ 被连接并通过一个全连接层表示为 $f(\tilde{s}_ t, \tilde{z}_ t)$。然后应用一个sigmoid函数作为激活信号来控制门，公式为：

$$
\delta_ t(w_ t = 1|\tilde{s}_ t, \tilde{z}_ t) =
\begin{cases}
1, & \text{如果}\ sigmoid(f(\tilde{s}_ t, \tilde{z}_ t)) \geq 0.5, \\
0, & \text{否则}.
\end{cases}
$$

当门检测到可控和非可控状态之间的依赖（ $w_ t = 1$ ）时，后续的非可控状态 $\tilde{z}_ {t+1}$ 通过同时考虑 $\tilde{s}_ t$ 和 $\tilde{z}_ t$ 来确定，使用动作自由转换定义如下：

$$
\tilde{z}_ {t+1} \sim p(\tilde{z}_ {t+1} | \tilde{z}_ t, w_ t \odot \tilde{s}_ t).
$$

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/c779e222786c4c339e8bfd2e55aa2d55.jpeg" width="800" />
</div>


### B. 基于隔离想象的的行为学习
得益于解耦的世界模型，我们可以优化代理行为，以适应性地考虑可用动作与潜在未来状态的非可控动态之间的关系。一个实际的例子是自动驾驶，其中其他车辆的运动可以自然地被视为非可控但可预测的组成部分。如图6所示，我们这里提出了一种改进的演员-评论家学习算法，它（i）允许动作自由分支在动作条件分支之前预见未来，并（ii）利用非可控动态的预测未来信息做出更具前瞻性的决策。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/f740cc3308254f0f96bf520d4f70b33a.jpeg" width="800" />
</div>



假设我们在想象阶段的时间步 $t$ 做出决策。原始Dreamer 方法的直接解决方案是学习基于隔离的可控状态 $\tilde{s}_ t \in \mathbb{R}^{1 \times d}$ 的动作模型和价值模型。借助注意力机制，我们可以在它和未来非可控状态之间建立联系。重要的是要注意，我们只在最初的想象步骤中使用稀疏依赖来获得 $\tilde{z}_ {t+1}$，因为此时后续的可控状态在时间步 $t$ 还不可用。一旦我们预测了未来非可控状态的序列 $\tilde{z}_ t:t+\tau \in \mathbb{R}^{\tau \times d}$，其中 $\tau$ 是从当前时间起滑动窗口的长度，我们就使用以下方程明确计算它们之间的关系：

$$
e_ t = \text{softmax}(\tilde{s}_ t \tilde{z}_ t^\top:t+\tau) \tilde{z}_ t:t+\tau + \tilde{s}_ t.
$$

这个方程允许我们使用注意力机制动态调整未来非可控状态的视野。通过这种方式，$\tilde{s}_ t$ 演变为一个更具“远见”的表示 $e_ t \in \mathbb{R}^{1 \times d}$。我们修改了 Dreamer 中的动作和价值模型如下：
动作模型： $a_ t \sim \pi(a_ t|e_ t)$,
价值模型： $v_ \xi(e_ t) \approx \mathbb{E}_ {\pi(\cdot|e_ t)}^\tau \sum_ {k=t}^{t+L} \gamma^{k-t} r_ k$,
其中 $L$ 是想象时间视野。如算法1所示，在想象过程中，我们首先使用动作自由转换模型获得长度为 $L + \tau$ 的非可控状态序列，表示为 $\{\tilde{z}_ i\}_ {i=t}^{t+L+\tau}$。在想象阶段的每个时间步，代理从远见状态 $e_ j$ 中抽取动作 $a_ j$，这是由 (13) 派生的。动作条件分支使用动作 $a_ j$ 在潜在想象中，并预测下一个可控状态 $s_ {j+1}$。我们遵循 DreamerV2 的方法使用 $\lambda$-return 作为训练目标来训练我们的动作模型，而我们的价值模型被训练来执行 $\lambda$-return 的回归。有关损失函数的更多信息，请参考 DreamerV2 论文中的 (5)-(6)。

### C. 通过展开非可控动态进行策略部署
在策略部署期间，如算法1中的行22-24所示，动作自由分支使用 (12) 预测下一步非可控状态 $\tilde{z}_ {t+1}$，然后从 $\tilde{z}_ {t+1}$ 开始连续展开未来的非可控状态 $\tilde{z}_ {t+2}:t+\tau$。类似于行为学习过程中使用的 (13)，学习到的未来状态注意力网络被用来适应性地整合 $s_ t$、$z_ t$ 和 $\tilde{z}_ {t+1}:t+\tau$。基于整合的特征 $e_ t$，Iso-Dream++ 代理然后从动作模型中抽取 $a_ t$ 与环境进行交互。如第II-B节中讨论的，如果非可控动态与控制任务无关，则在与环境交互时，每个时间步 $t$ 的策略仅使用可控动态的状态生成。

# IV. 实验
## A. 实验设置
### 基准测试：我们在两个强化学习环境上评估 Iso-Dream++：
- **CARLA**：CARLA 是一个具有复杂和逼真视觉观测的自动驾驶研究模拟器。我们训练我们的模型在“Town04”中执行第一人称高速公路驾驶任务，目标是在1000个时间步内尽可能远地驾驶，同时避免与任何30辆其他移动车辆或障碍物发生碰撞。除了我们的会议论文中的设置，我们还在研究中加入了更多多样化的设置，包括图7中所示的白天和夜晚模式。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/6a4252d6b1a8402fb1e5780f449e3133.jpeg" width="800" />
</div>



- **DeepMind Control Suite (DMC)**：DMC 包含一组连续控制任务，是视觉基础强化学习的标准基准。为了通过解耦不同组件来评估我们方法的泛化能力，我们使用了两个修改后的基准测试 [20]，即 video_ easy（包含10个简单视频）和 video_ hard（包含100个复杂视频）。

### 比较方法：我们与以下视觉强化学习方法进行比较：
- DreamerV2 [8]：一种基于模型的强化学习方法，直接从世界模型中的潜在变量中学习。潜在表示允许代理并行想象数千条轨迹。
- DreamerV3 [21]：Dreamer 的进一步改进版本，学习使用固定的超参数掌握多样化的领域。
- DreamerPro [22]：一种非对比的、无需重建的基于模型的强化学习方法，结合了 Dreamer [6] 和原型以增强对干扰的鲁棒性。
- CURL [23]：一种模型自由的强化学习方法，使用对比学习从原始像素中提取高级特征，最大化相同观测的增强数据之间的一致性。
- SVEA [24]：一种用于深度 Q-学习算法中的数据增强框架，提高了离策略强化学习的稳定性和泛化性。
- SAC [25]：一种模型自由的演员-评论家方法，优化了一个随机策略。
- DBC [26]：一种学习方法，学习了一个双射度量表示，没有重建损失。这种表示对观测中不同的任务无关细节是不变的。
- Denoised-MDP [27]：一个框架，将野外的信息分类为四种类型，基于可控性和与奖励的关系，并将有用信息表述为既可控又与奖励相关。

## B. CARLA 自动驾驶环境
### 实现：

在自动驾驶任务中，我们使用了一个安装在自车顶部、视角为60度的摄像头，获取64×64像素的图像。遵循 DBC [26] 中的设置，为了鼓励高速公路进展并对碰撞进行惩罚，奖励被制定为 $r_ t = v_ {ego}^T \hat{u}_ h \cdot \Delta t - \xi_ 1 \cdot (1 - \xi_ 2 \cdot |\text{steer}|)$，其中 $v_ {ego}$ 是自车的速度向量，投影到高速公路的单位向量 $\hat{u}_ h$ 上，并乘以时间离散化 $\Delta t = 0.05$ 来以米为单位测量高速公路进展。我们对碰撞使用 $1 \in \mathbb{R}^+$，并且使用转向惩罚 $\text{steer} \in [-1, 1]$ 来促进车道保持。超参数设置为 $\xi_ 1 = 10^{-4}$ 和 $\xi_ 2 = 1$。我们在 (6) 中使用 $\beta_ 1 = \beta_ 2 = 1$ 和 $\alpha = 1$，在 (10) 中使用 $\lambda_ 1 = \lambda_ 2 = 1$，在 (13) 中使用 $\tau = 5$。

### 定量比较：

我们在图8(a)中展示了 CARLA 中的定量结果。Iso-Dream++ 显著优于比较模型，包括 DreamerV2、DreamerV3、DreamerPro 和 Denoised-MDP。在50万环境步骤后，Iso-Dream++ 实现了大约60的平均回报，而 DreamerV2 和 Denoised-MDP 分别实现了10和25。在 DreamerV2 中，潜在表示包含了可控和非可控动态，这增加了在想象中建模状态转换的复杂性。与 Denoised-MDP 相比，后者也根据可控性解耦信息，Iso-Dream++ 通过展开未来非可控状态来做出前瞻性决策的优势。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/e5cb9ac86aee4063aeb10b03749796d0.jpeg" width="800" />
</div>



### 消融研究：

图8(b)提供了消融研究结果，验证了逆动力学、非可控状态的展开策略、注意力机制以及静态信息建模的有效性。如图中的绿色曲线所示，移除逆向单元会降低 IsoDream++ 的性能，这表明了隔离可控和非可控组件的重要性。对于图中红色曲线所示的模型，我们没有使用未来非可控状态的展开作为动作模型的输入。结果表明，通过提前感知潜在风险，展开非可控状态的行动自由分支显著改善了代理的决策结果。此外，我们评估了没有注意力机制的 Iso-Dream++，其中动作模型直接将当前可控状态与未来非可控状态的序列串联起来并将其作为输入。如图中的紫色曲线所示，注意力机制比串联更好地从未来非可控动态中提取有价值信息。此外，如图中的棕色曲线所示，我们的方法在没有单独网络分支来捕获静态信息时性能下降了约15%。此外，蓝色曲线和橙色曲线之间的比较揭示了，当我们移除最小-最大方差约束和稀疏依赖建模时，我们模型的性能下降。与 DMC 套件不同，在 CARLA 环境中，原始 Iso-Dream 对训练崩溃更加脆弱，稀疏依赖建模方法在 Iso-Dream++ 的改进性能中起着至关重要的作用。在图9中，我们展示了有无稀疏依赖的模型产生的视觉效果。没有稀疏依赖（顶行），代理在前方有很多车辆时倾向于跟随前面的车辆。在图的底行中，代理可以灵活地超车和加速。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/d179b77f63a3424d8c3ae07b17876061.jpeg" width="800" />
</div>


### 定性结果：

我们在图10中展示了 Iso-Dream++ 在 CARLA 环境中的视频预测结果。由于该环境中的第一人称视角，代理动作可能影响观测中的所有像素值，因为安装在主车上的摄像头（即代理）会移动。因此，我们可以将其他车辆的动态视为可控和非可控状态的组合。相应地，我们的模型通过学习动作条件分支和动作自由分支之间的注意力掩码值来确定哪个组件占主导地位。“动作自由掩码”在其他车辆周围呈现热点，而相应区域上“动作条件掩码”的注意力值仍然大于零。如图的第三和第五行所示，Iso-Dream++ 主要学习了动作条件分支中的山脉和树木的动态，以及动作自由分支中的其他驾驶车辆的动态，这有助于代理通过展开非可控组件来预览其他车辆的可能未来状态，从而避免碰撞。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/c78c6e44670b488b9e0f24d56250e077.jpeg" width="800" />
</div>



### 超参数 $\tau$ 的分析：

我们评估了使用不同数量的非可控状态展开步骤作为动作模型输入的效果。从图11中的结果，我们观察到我们的模型在 $\tau = 5$ 时表现最佳。然而，在 $\tau \in [5, 10, 15]$ 之间没有显著差异，因为非可控状态的长期预测可能会增加模型误差。此外，我们实现了一个没有将非可控状态展开到未来的模型，即 $\tau = 0$。它的性能显著低于其他基线 $\tau \in [5, 10, 15]$，这证明了在策略优化中展开解耦的动作自由分支的好处。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/c9a1c7a7c0aa4711a1ffae48e87409b4.jpeg" width="800" />
</div>



## C. DeepMind 控制套件
### 实现：

我们在 DMC 的 video_ easy 和 video_ hard 基准测试上评估我们的模型，其中背景在整个情节中不断变化。所有实验仅使用视觉观测，形状为 64×64×3。情节持续1000步，并且有随机化的初始状态。我们在任务中应用了一个固定的行动重复 R = 2。在这个环境中，由于背景被真实世界的视频随机替换，背景的非可控运动会影响动态学习和行为学习的过程。因此，为了获得更好的决策策略并避免嘈杂背景的干扰，代理可能会在时空中解耦非可控表示（即动态背景）和可控表示，并且只使用可控表示进行控制，从而去除建模稀疏依赖。与我们的初步工作 [1] 不同，我们没有仅使用重建损失训练动作自由分支，而是遵循第 III-A 节中描述的结构，因为某些视频背景中的非可控动态对于学习来说过于复杂，特别是在 video_ hard 基准测试中。我们使用4个任务评估模型，即 Finger Spin、Cheetah Run、Walker Walk 和 Hopper Stand。环境步数的最大数量为50万。我们在 (6) 中使用 $\beta_ 1 = \beta_ 2 = 1$ 和 $\alpha = 1$，在 (10) 中使用 $\lambda_ 1 = \lambda_ 2 = 1$。

### 定量比较：

我们在表 I 中展示了 Iso-Dream++ 在 video_ easy 基准测试中的定量结果。我们的最终模型在所有任务中显著优于 DreamerV2 和其他基线。与 DBC 和 Denoised-MDP 相比，这两种方法都旨在从复杂的视觉干扰中提取任务相关表示，我们的方法在所有四个任务中都具有更大的性能提升，表明通过模块化结构通过解耦不同动态来解耦不同动态，并通过方差约束提供更清晰和有用的信息，对于下游任务更为强大。此外，我们的表现优于 DreamerPro，后者也基于 Dreamer 但学习世界模型而无需重建观测。这表明我们的模型有效地帮助代理学习可控视觉表示，并减轻复杂背景干扰。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/904ca16d24ca4084836713178f6bdd57.jpeg" width="800" />
</div>


### 分析最小-最大方差约束：

我们通过从 Iso-Dream++ 的训练过程中移除它来研究第 III-A2 节中提出的方差约束的有效性。如图 12 所示，比较的模型是针对 10 个种子进行训练的，我们提出的方法在大多数任务中提高了我们模型的性能，特别是在手指旋转任务中，我们见证了显著的训练崩溃（见图 4）。在图 13 中，我们提供了有无提出的方差约束训练的模型之间的定性比较。比较第五和第六行的动作自由分支输出，我们观察到动作自由动态（例如湖上的灯光）被方差约束正确地分配给动作自由分支，防止动作条件分支捕获所有动态信息，即训练崩溃。由于动作条件分支捕获的纯净动态，我们的模型在有无方差约束的情况下获得了明确的改进。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/dd6149532bd040c8b22135c91fed2164.jpeg" width="800" />
</div>


<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/90ec303c70824834a67f7896aa7b010b.jpeg" width="800" />
</div>



<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/07a62b7ae23c44cebb05dae260f090d1.jpeg" width="800" />
</div>


### 定性结果：

我们使用 Iso-Dream++ 在 DMC 环境中执行视频预测，其中背景是 video_ easy。帧序列和动作是从测试情节中随机收集的。前 5 帧提供给模型，接下来的 45 帧仅基于动作输入进行预测。在这个环境中，视频背景可以被视为非可控动态和静态表示的组合。图 14 可视化了整个生成的 RGB 图像、解耦的 RGB 组件以及三个网络分支的相应掩码。从这些结果中，我们观察到我们的方法有能力预测长期序列，并从复杂视觉图像中解耦可控（代理）和非可控动态（背景运动）。如图 14 的第三和第四行所示，可控表示已成功隔离并与其掩码匹配。如图的第五和第六行所示，火焰和海浪的运动被动作自由分支捕获为非可控动态。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/36eb4a3518ee42fabfec29a6c3106b8d.jpeg" width="800" />
</div>


### 对即时干扰的鲁棒性：

为了评估 Iso-Dream++ 抵抗即时视觉干扰的能力，我们在 video_ easy 基准上训练了强化学习模型，并在（i）video_ hard；（ii）带有高斯噪声的视频_ easy 上评估了它们。在表 II 中，我们比较了 Iso-Dream++ 与 DreamerPro 和 Denoised-MDP 的结果，这两种方法都专注于学习对视觉噪声具有鲁棒性的表示。我们观察到 Iso-Dream++ 在意外干扰面前具有显著优势，一致性地在所有任务中优于 DreamerPro 和 Denoised-MDP。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/6d201b723f3d4a4fb5932bebe5eebc6c.jpeg" width="800" />
</div>


## D. 迁移学习分析
### CARLA 中非可控动态的迁移：

我们的模型在不同分支中学习了不同的动态，这使得它自然适合于迁移学习。与通常从预训练的源任务转移所有知识的常见方法不同，我们可以有选择地转移特定知识以适应目标任务。具体来说，我们只转移相关的知识，例如源任务和目标任务之间共享的动态，以实现精确的解耦和在目标任务上的鲁棒决策。在图 7 中，我们可以看到日夜模式之间的非可控动态是相似的，即其他驾驶车辆的运动。我们保持在 CARLA 环境的白天模式上预训练的动作自由分支，并将其训练到夜晚模式。结果如图 15 所示。比较橙色曲线和蓝色曲线，我们的模型通过转移动作自由分支中的非可控动态有了显著改进。然而，DreamerV2 的性能提升很小。因此，由于我们 Iso-Dream++ 中的模块化结构，当存在两个具有相似动态的环境时，我们可以先在简单的环境上进行训练，然后加载特定的预训练分支来帮助模型在困难任务上学习。因此，我们 Iso-Dream++ 的模块化结构允许我们根据对领域差距的先验知识，有选择地转移可控或非可控部分到新领域。

<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/63219009670a446bacfed0ca007608d5.jpeg" width="800" />
</div>



### DMC 中可控动态的迁移：

对于 DMC，我们使用 video_ easy（源）和 video_ hard（目标）基准来评估我们模型的迁移能力。我们转移动作条件分支中的可控信息，因为两个环境中的可控动态是相同的，即代理的运动。从图 16 中，我们有两个关键观察。首先，加载预训练的动作条件分支后，Iso-Dream++ 显示出比非预训练对应物显著的优势。其次，值得注意的是，通过预训练实现的性能提升，我们的模型超过了 DreamerV2 相当大的幅度。
<div align=center>
   <img src="https://img-blog.csdnimg.cn/direct/06d216e6eb74447389b131e00d85ee91.jpeg" width="800" />
</div>

# VI. 结论
在本文中，我们提出了一个名为 Iso-Dream++ 的基于模型的强化学习（MBRL）框架，主要解决了在复杂视觉动态存在时基于视觉的预测和控制的困难。我们的方法在世界模型表示学习和相应的 MBRL 算法中有四个新颖的贡献。首先，它通过模块化网络结构和逆动力学学习解耦可控和非可控潜在状态转换。其次，它引入了最小-最大方差约束来防止“训练崩溃”，即单个状态转换分支捕获所有信息。第三，它通过将非可控动态展开到未来并学习它们对当前行为的影响来做长期决策。第四，它模拟了未来非可控动态对当前可控动态的稀疏依赖性，以处理一些实际的动态环境。Iso-Dream++ 在 CARLA 自动驾驶任务上取得了竞争性的结果，在该任务中，其他车辆可以自然地被视为非可控组件，表明借助解耦的潜在状态，代理可以通过预览动作自由网络分支中可能的未来状态来做出更具前瞻性的决策。此外，我们的方法在修改后的 DeepMind Control Suite 中有效改进了视觉控制任务，在标准、嘈杂和迁移学习设置中比现有方法具有显著优势。

# 声明
本文内容为论文学习收获分享，受限于知识能力，本文对原文的理解可能存在偏差，最终内容以原论文为准备。  
本文信息旨在传播和交流学术，其内容由作者负责，不代表本号观点。文中内容如涉及作品文字。图片等内容、版权和其他问题，请及时与我们联系，我们将在第一时间删文处理。
