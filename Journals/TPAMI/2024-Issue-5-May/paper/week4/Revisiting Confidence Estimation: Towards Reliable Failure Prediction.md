# 题目：[Revisiting Confidence Estimation: Towards Reliable Failure Prediction](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10356834)  
## 重新审视置信度估计：迈向可靠的故障预测
**作者：Fei Zhu；Xu-Yao Zhang；Zhen Cheng；Cheng-Lin Liu** 

****
# 摘要
可靠的置信度估计在许多风险敏感的应用中是一项具有挑战性但至关重要的需求。然而，现代深度神经网络通常对其错误预测过于自信，即对已知类别的错误分类样本和未知类别的分布外（OOD）样本过于自信。近年来，已经开发了许多置信度校准和OOD检测方法。在本文中，我们发现了一个普遍存在但实际上被忽视的现象，即大多数置信度估计方法对于检测错误分类有害。我们调查了这个问题，揭示了流行的校准和OOD检测方法通常导致正确分类和错误分类样本之间的置信度分离更差，使得很难决定是否信任预测。最后，我们提出通过寻找平坦极小值来扩大置信度差距，这在包括平衡、长尾和协变量偏移分类场景的各种设置下实现了最先进的故障预测性能。我们的研究不仅为可靠的置信度估计提供了一个强大的基线，还在理解校准、OOD检测和故障预测之间架起了桥梁。

# 关键词
- 置信度估计
- 不确定性量化
- 故障预测
- 错误分类检测
- 选择性分类
- 分布外检测
- 置信度校准
- 模型可靠性
- 可信赖性
- 平坦极小值

## I. 引言
深度神经网络（DNN），尤其是视觉模型，已经广泛应用于安全关键的应用中，例如计算机辅助医学诊断、自动驾驶和机器人技术。对于这些应用，除了预测准确性，另一个关键要求是为用户提供可靠的置信度以做出安全决策。例如，当检测网络无法自信地预测障碍物时，自动驾驶汽车应更多地依赖其他传感器或触发警报。另一个例子是，当疾病诊断网络的置信度较低时，应将控制权交给人类医生。不幸的是，现代DNN通常对其错误预测过于自信，即对训练类别的错误分类样本和未知类别的OOD样本分配高置信度。过度自信的问题使DNN不可信，在实际应用中部署时带来了很大的担忧。

近年来，已经提出了许多置信度估计方法，使DNN能够为其预测提供可靠的置信度。大多数这些方法集中于两个具体任务，即置信度校准和OOD检测。（1）置信度校准通过匹配准确性和置信度分数来缓解过度自信问题并反映预测不确定性。一类方法旨在在训练过程中学习良好校准的模型。另一类方法使用后处理技术来校准DNN。（2）OOD检测则集中于基于模型置信度判断输入样本是否来自未见过的类别。大多数现有方法以后处理方式解决OOD检测问题，其他工作则集中于在训练时学习具有更好OOD检测能力的模型。如果我们关注置信度校准和OOD检测取得的进展，似乎DNN的置信度估计已经得到很好的解决，因为分布内（InD）和OOD样本的置信度都得到了很好的校准。

在本文中，我们研究了一个自然但被忽视的问题：我们能否通过过滤低置信度预测来检测错误分类的样本？这或许是评估估计置信度可靠性的最直接和实际的方法。实际上，这个问题在文献中被研究为故障预测（或错误分类检测），其目的是检测从已见类别中错误分类的自然样本（例如，测试集中错误分类的样本）。在实践中，对于一个已部署的系统，大多数输入来自已见类别，并且广泛存在错误分类错误。因此，故障预测是开发可靠和可信赖的机器学习系统的一种高度实际和有价值的工具。然而，与广泛研究的置信度校准和OOD检测相比，关于故障预测的工作较少。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/3fe9c62aca484921933f0223fd2a99ac.png)



我们从故障预测的角度重新审视当前的置信度估计方法，出于以下原因：首先，校准和故障预测都集中在来自已知类别的分布内数据，并且共享相同的动机，即使模型能够提供可靠的置信度以做出安全决策。从实际的角度来看，拥有一个校准的分类器，一种验证其可信度的自然方法是过滤掉低置信度的预测。其次，现有的OOD检测方法旨在通过为InD样本分配高置信度和为OOD样本分配低置信度来分离InD样本和OOD样本。然而，在实践中，错误分类的样本也应具有低置信度，以便与正确样本区分开来。换句话说，OOD检测和故障预测应以统一的方式进行考虑，一个好的置信度估计器应有助于同时检测OOD和错误分类的InD样本。第三，校准和OOD检测已引起机器学习社区的显著关注。然而，很少有关于故障预测的工作，这是一个实际、重要但尚未被充分认识的研究领域，它代表了一个自然的试验台，用于评估给定方法的置信度估计的可靠性。从故障预测的角度重新审视置信度估计方法不仅有助于我们理解现有方法的有效性，还促进了故障预测的研究。

尽管社区中的普遍看法认为，通过校准或OOD检测方法估计的置信度对故障预测有用，但我们发现了一个令人惊讶的病理现象：许多流行的校准和OOD检测方法（包括训练时和后处理方法）在故障预测上通常具有负面影响。

最后，我们如何提高DNN的故障预测性能？一方面，故障预测需要更好地区分正确分类和错误分类样本的置信度，这将增加更改正确分类样本为错误分类的难度，因为置信度边距较大。有趣的是，这与DNN中的“平坦性”概念密切相关，该概念反映了扰动模型参数时正确分类样本变为错误分类样本的敏感性。另一方面，我们观察到一个有趣的可靠过拟合现象，即在模型训练过程中，故障预测性能很容易过拟合。受平坦极小值和平坦置信度分离效果的启发，我们提出一个简单的假设：平坦极小值对故障预测有利。我们通过广泛的实验验证了这一假设，并提出了一种基于平坦极小值的简单技术，能够实现最先进的置信度估计性能。

总的来说，我们的贡献如下：

- 我们通过评估流行的校准和OOD检测方法在具有挑战性和实用性的故障预测任务中的表现，重新思考了置信度的可靠性。令人惊讶的是，我们发现这些方法对故障预测往往有负面影响。
- 我们从适当的评分规则和贝叶斯最优拒绝规则的角度，提供了关于校准和OOD检测对故障预测的详细分析和讨论。
- 我们揭示了一个有趣的可靠过拟合现象，即故障预测性能在训练过程中很容易过拟合。这种现象存在于不同的模型和数据集设置中。
- 我们提出寻找平坦极小值，以显著降低错误分类样本的置信度，同时保持正确样本的置信度。为此，提出了一种基于平坦极小值的简单技术。
- 在平衡、长尾和协变量偏移分类场景中的广泛实验表明，我们的方法实现了最先进的置信度估计性能。

本文扩展了我们之前的会议发表，主要体现在五个方面：（1）通过全面的实验展示了流行OOD检测方法在故障预测中的行为的新见解。（2）基于适当的评分规则和贝叶斯最优拒绝规则，从更多的理论角度提供了关于校准和OOD检测方法在检测错误分类实例中的失败的分析。（3）基于PAC贝叶斯框架，提供了平坦极小值对提高置信度估计的理论分析。（4）设计了更具挑战性和现实性的基准，即长尾和协变量偏移场景下的置信度估计，我们的方法在这些场景中也达到了最先进的性能。（5）在标准OOD检测基准上的实验展示了我们的方法的强大OOD检测能力。总之，我们从故障预测的角度重新思考了当前校准和OOD检测方法的可靠性。我们的发现很重要，因为它可以更好地评估最近在置信度估计领域的进展。最后，我们提供了一个强大且统一的基线，能够提高校准性能并检测错误分类和OOD样本。

本文其余部分的组织如下：第二部分介绍了置信度校准、OOD检测和故障预测的相关问题表述和背景。第三部分评估并分析了流行校准和OOD检测方法对故障预测的影响。第四部分展示了在平衡、长尾和协变量偏移分类场景下，通过寻找平坦极小值可以显著提高故障预测性能。第五部分提供了总结。

## II. 问题的表述和背景
### 多类别分类
考虑一个 $K$ 类分类任务，设 $(X,Y) \in \mathcal{X} \times \mathcal{Y}$ 为联合分布的随机变量，其中 $\mathcal{X} \subset \mathbb{R}^d$ 表示分布内特征空间， $\mathcal{Y}$ 是标签空间。具体而言， $\mathcal{Y} = \{e_1, \ldots, e_K\}$，其中 $e_k \in \{0, 1\}^K$ 是在第 $k$ 个索引处为1，其余为0的独热向量。我们给定从 $(\mathcal{X}, \mathcal{Y})$ 独立同分布抽取的标记样本 $D = \{(x_i, y_i)\}_{i=1}^n$，其密度为 $p(x, y)$。我们假设标签是根据真实后验分布 $Q = (Q_1, \ldots, Q_K) \in \Delta^K$ 抽取的，其中 $Q_k := P(Y = e_k \mid X)$ 且 $\Delta^K$ 是概率单纯形 $\Delta^K = \{(p_1, \ldots, p_K) \in [0, 1]^K : \sum_k p_k = 1\}$。DNN分类器 $f$ 预测输入样本 $x$ 的类别

$$
f(x) = \arg\max_{k=1, \ldots, K} p_k(x),
$$

$$
p_k(x) = \frac{\exp(z_k(x))}{\sum_{k'=1}^K \exp(z_{k'}(x))},
$$

其中 $z_k(x)$ 是网络关于类别 $k$ 的对数值输出， $p_k(x)$ 是 $x$ 属于类别 $k$ 的概率（对对数值进行softmax）。分类器 $f$ 的分类风险可以相对于0-1损失定义为

$$
R_{0-1}(f) = \mathbb{E}_{p(x,y)}[\mathbb{I}(f(x) \neq y)],
$$

其中 $\mathbb{I}$ 是指示函数。贝叶斯最优分类器 $f^*$ 可以定义如下：

**定义1（贝叶斯最优分类器）:** 多类分类的贝叶斯最优解 $f^* = \arg\min_f R_{0-1}(f)$ 可以表示为 $f^*(x) = \arg\max_{y \in \mathcal{Y}} P(y \mid x)$。

在实践中，(2) 中的风险不是容易最小化的，因为训练样本数量是有限的，并且最小化零一损失已知在计算上是不可行的。因此，我们通常最小化经验替代风险。具体而言，令 $\ell$ 为替代损失（例如对数损失），基于经验风险最小化方法，最小化以下经验替代风险： $R_\ell(f) = \frac{1}{n} \sum_{i=1}^n [\ell(f(x), y)]$，其中可以添加正则化以避免过拟合。在推理阶段， $\hat{y} = f(x)$ 可以作为预测类别返回，并且关联的概率分数 $p_{\hat{y}}(x)$，即最大softmax概率（MSP），可以视为预测的置信度。除了直接使用MSP作为置信度外，还可以使用其他分数函数（例如熵、能量分数或maxlogit分数）。

### A. 置信度校准
置信度校准集中于InD样本，旨在校准模型的置信度以指示实际的正确性可能性。例如，如果一个校准的模型预测一组输入属于类别 $y$ 的概率为40%，那么我们期望40%的输入确实属于类别 $y$。形式上，表示分数 $S = f(X)$， $S = (S_1, \ldots, S_K) \in \Delta^K$ 为描述概率分类器 $f$ 输出分数的随机向量，则我们声明：

**问题1（校准）:** 如果对于得到分数 $s$ 的实例，类别概率等于 $s$，则给出分数 $s = (s_1, \ldots, s_k)$ 的分类器是联合校准的

$$
P(Y = e_k \mid S = s) = s_k \text{ 对于 } k = 1, \ldots, K.
$$

在实践中，估计 $Y$ 在 $S$ 条件下的概率是具有挑战性的，因为样本数量有限。因此，在机器学习界，常用的校准概念是一个较弱的定义：如果对于预测类别的置信度分数为 $s$ 的实例，预测正确的概率是 $s$： $P(Y = e_{\arg\max(s)} \mid \max(S) = s) = s$。

最常用的校准估计量是期望校准误差（ECE），通过在 [0,1] 下将置信度分为 M 个等间隔区间 $\{B_m\}_{m=1}^M$ 来近似错校准。然后通过以下方式估计错校准

$$
ECE = \sum_{m=1}^M \frac{|B_m|}{n} |acc(B_m) - conf(B_m)|,
$$

其中 $n$ 是所有样本的数量。ECE的替代品包括负对数似然（NLL）和Brier分数。

#### 改进校准
已经提出了许多策略来解决现代DNN的错校准问题。（1）一类方法旨在在训练期间学习良好校准的模型。例如，一些工作发现，使用mixup训练的DNN的预测分数校准更好。Muller等人展示了标签平滑的有利校准效果。Mukhoti等人表明，焦点损失可以自动学习良好校准的模型。（2）另一类方法以后处理方式重新调整预测。在这些方法中，温度缩放（TS）是一种有效且简单的技术，已经激发了各种后处理方法。

#### 校准的经验研究
除了校准策略外，还有一些关于校准的经验研究。例如，Ovadia等人研究了分布偏移下的校准，经验上发现不同校准方法在分布偏移下普遍存在的性能下降。Minderer等人发现，最新的非卷积模型是校准良好的，这表明架构是校准的一个重要因素。

### B. OOD检测
OOD检测的目标是在测试时拒绝OOD样本。形式上，我们有一个InD联合分布 $D_{\mathcal{X}\_{\text{in}}\mathcal{Y}\_{\text{in}}}$，其中 $\mathcal{X}\_{\text{in}} \in \mathcal{X}$ 和 $\mathcal{Y}\_{\text{in}} \in \mathcal{Y}$ 是随机变量。我们也有一个OOD联合分布 $D_{\mathcal{X}\_{\text{out}}\mathcal{Y}\_{\text{out}}}$，其中 $\mathcal{X}\_{\text{out}} \in \mathbb{R}^d \setminus \mathcal{X}$ 和 $\mathcal{Y}\_{\text{out}}$ 不在 $\mathcal{Y}$ 中。在测试时，我们遇到InD和OOD联合分布的混合： $D_{\mathcal{X}\mathcal{Y}} = \pi_{\text{in}} D_{\mathcal{X}\_{\text{in}}\mathcal{Y}\_{\text{in}}} + (1 - \pi_{\text{in}}) D_{\mathcal{X}\_{\text{out}}\mathcal{Y}\_{\text{out}}}$，并且只能观察到边际分布 $D_{\mathcal{X}} = \pi_{\text{in}} D_{\mathcal{X}\_{\text{in}}} + (1 - \pi_{\text{in}}) D_{\mathcal{X}\_{\text{out}}}$，其中 $\pi_{\text{in}} \in (0, 1)$ 是一个未知的先验概率。

**备注1:** 与现有工作的主流一致，我们本文中的OOD检测是指检测与训练类别无重叠标签的新类（语义）偏移示例。在开放环境中，OOD示例可能来自各个领域，可能远离或靠近InD。为了使检测OOD示例成为可能，我们假设 $\mathcal{X}\_{\text{out}}$ 和 $\mathcal{X}\_{\text{in}}$ 位于输入空间 $\mathbb{R}^d$ 的不同子集。然而，更广泛的OOD检测可能涉及检测协变量偏移示例，在这种情况下，OOD的定义可能会有所不同。

**问题2（OOD检测）:** 对于在从InD联合分布 $D_{\mathcal{X}\_{\text{in}}\mathcal{Y}\_{\text{in}}}$ 独立同分布抽取的训练数据上训练的分类器 $f$，给定分数函数 $s$ 和预定义阈值 $\delta$，OOD检测的目的是基于决策函数 $g: \mathcal{X} \to \{0, 1\}$ 拒绝OOD样本，以使从混合边际分布 $D_{\mathcal{X}}$ 抽取的任何测试数据 $x$：如果 $s(x) \geq \delta$，则 $g(x) = 1$（内样本， $x \in D_{\mathcal{X}\_{\text{in}}}$），否则 $g(x) = 0$（外样本， $x \in D_{\mathcal{X}\_{\text{out}}}$）。

**定义2（OOD检测风险）:** 给定混合分布 $p(x) = \pi_{\text{in}} p(x \mid \text{in}) + (1 - \pi_{\text{in}}) p(x \mid \text{out})$，OOD检测的风险是

$$
R_{\text{ood}}(g) = \pi_{\text{in}} \mathbb{E}_ {p(x \mid \text{in})}[\mathbb{I}(g(x) = 0)] + (1 - \pi_{\text{in}}) \mathbb{E}_{p(x \mid \text{out})}[\mathbb{I}(g(x) = 1)]。
$$

上述定义表明，OOD检测的风险来自于将内样本（已见类别）拒绝为外样本（未见类别），以及接受外样本为内样本。值得一提的是，(4) 中的风险无法直接优化，因为一个普遍的假设是，OOD样本仅在推理阶段遇到，并且在训练阶段不可用，这使得OOD检测成为一个具有挑战性的任务。

OOD检测的常用指标是95%真阳性率下的假阳性率（FPR95），接收者操作特性曲线下面积（AUROC）和精确率-召回率曲线下面积（AUPR）。例如，AUROC是一个与阈值无关的指标，反映了内样本和外样本之间的排名性能。

#### 改进OOD检测
OOD检测吸引了两个方向的研究兴趣，即事后和训练时正则化。（1）一些工作集中于设计更有效的评分函数以检测OOD样本，例如ODIN评分、马氏距离评分、能量评分、GradNorm评分、ReAct和MLogit评分。（2）其他方法通过训练时正则化解决OOD检测问题。例如，Hendrycks等人利用异常样本训练OOD检测器。Wei等人表明，Logit归一化可以缓解OOD样本的过度自信问题。

### C. 故障预测
故障预测，也称为错误分类检测或选择性分类，集中于基于置信度排名区分错误 $(D_{\mathcal{X}\_{\text{in}}}^{\times})$ 和正确 $(D_{\mathcal{X}\_{\text{in}}}^{\checkmark})$ 的预测。直观上，如果每个错误分类样本的关联置信度低于任何正确分类样本的置信度，我们就可以成功预测分类模型所犯的每个错误。

**问题3（故障预测）:** 对于在从InD联合分布 $D_{\mathcal{X}\_{\text{in}}\mathcal{Y}\_{\text{in}}}$ 独立同分布抽取的训练数据上训练的分类器 $f$，给定分数函数 $s$ 和预定义阈值 $\delta$，故障预测的目的是基于以下决策函数 $g: \mathcal{X} \to \{0, 1\}$ 拒绝错误分类的InD样本，以使从InD $D_{\mathcal{X}\_{\text{in}}}$ 抽取的任何测试数据 $x$：如果 $s(x) \geq \delta$，则 $g(x) = 1$（正确， $x \in D_{\mathcal{X}\_{\text{in}}}^{\checkmark}$），否则 $g(x) = 0$（错误分类， $x \in D_{\mathcal{X}\_{\text{in}}}^{\times}$）。

**定义3（故障预测风险）:** 给定分类器 $f$ 和决策函数 $g$，故障预测的风险是

$$
R_{\text{fp}}(f, g) = \mathbb{E}_{p(x \mid \text{in})}[c \cdot \mathbb{I}(g(x) = 0) + \mathbb{I}(f(x) \neq y) \cdot \mathbb{I}(g(x) = 1)]，
$$

其中 $c \in (0, 1)$ 是拒绝成本。同样地，(5) 中的风险无法直接优化，因为DNN通常具有近乎完美的训练准确率，并且存在很少的错误分类训练样本。

故障预测的常用指标包括风险覆盖曲线下面积（AURC），标准化AURC（E-AURC），FPR95，AUROC，成功精确率-召回率曲线下面积（AUPR-S）和错误精确率-召回率曲线下面积（AUPR-E）。特别是，已经表明最小化AURC指标等同于最小化 $R_{\text{fp}}(f, g)$。

#### 改进故障预测
对于DNN，Hendrycks等人首先通过使用MSP建立了故障预测的标准基线。Trust-Score采用分类器和最近邻分类器之间的相似性作为置信度度量。一些工作将故障预测公式化为一个监督的二元分类问题。具体而言，ConfidNet和SS通过学习训练集中的错误分类样本来训练辅助模型以预测置信度。最近，Qu等人提出了一个元学习框架，通过构建虚拟训练和测试集来训练辅助模型。然而，当模型具有高训练准确率时，这些方法可能会失败，因为训练集中可能存在很少或没有错误分类的样本。CRL通过正则化模型学习基于历史正确率的有序排名关系来改进故障预测。OpenMix表明，合理使用来自非目标类别的易得的异常样本可以显著帮助故障预测。对于图像增强和翻译领域的超分辨率和深度估计等回归任务，Upadhyay等人提出了BayesCap，该方法在后处理方式下为冻结模型学习贝叶斯身份映射。

## III. 校准和OOD检测是否有助于故障预测？
近年来，针对现代DNN的过度自信问题进行了大量研究，现有方法确实有助于DNN的校准和OOD检测。在本节中，我们实证研究了估计置信度在故障预测中的可靠性。

### A. 实验设置
数据集和网络架构。我们在基准数据集CIFAR-10和CIFAR-100，以及大规模ImageNet数据集上进行了全面的实验。在网络架构方面，我们考虑了一系列模型：在CIFAR-10和CIFAR-100上的PreAct-ResNet110、WideResNet、DenseNet。在ImageNet上，我们分别使用了ResNet-18和ResNet-50模型。

评估指标。我们采用标准指标来衡量故障预测：AURC、E-AURC、AUROC、FPR95、AUPR-S和AUPR-E。AURC、E-AURC、FPR95值越低，AUROC、AUPR-S、AUPR-E值越高，表示故障预测能力越强。此外，我们强调测试准确率也很重要，因为我们不能为了提高置信度估计的质量而牺牲原始任务的性能。补充材料提供了这些指标的详细定义。

实现细节。所有模型都使用动量为0.9的SGD进行训练，初始学习率为0.1，权重衰减为5e-4，在CIFAR-10和CIFAR-100上进行200个周期的训练，迷你批量大小为128。学习率在100和150个周期时分别降低10倍。我们采用标准的数据增强方案，即随机水平翻转和32x32随机裁剪（每侧填充4个像素后）。对于每个实验，报告三个随机运行的均值和标准差。

评估的置信度估计方法。（1）我们评估了各种校准方法，包括流行的训练时正则化方法，如mixup、标签平滑（LS）、焦点损失、Lp范数和后处理方法TS。这些方法已被验证对解决DNN的错校准问题有效。（2）我们评估了各种OOD检测方法，包括训练时正则化方法，如LogitNorm、异常暴露（OE）和后处理方法，如ODIN、能量、ReAct、MLogit。许多这些方法最近被提出，并显示出强大的OOD检测性能。补充材料提供了每种方法的介绍和超参数设置。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4dd300d459014d60ae10afeec61661f8.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/9aabfbe180a045f8b7ca3aef236db49e.png)


### B. 实验结果
在我们的实验中，我们确认了mixup、LS、Focal和Lp范数的正置信度校准效果。例如，在CIFAR-100上，使用焦点损失，ResNet110的ECE（%）可以从14.98降至5.66，DenseNet的ECE从8.00降至1.35。这些观察结果与文献中的一致。同样，在我们的实验中，评估的OOD检测方法确实对检测OOD样本有效。

#### 流行的校准方法可能会损害故障预测
在实践中，用户自然期望校准的置信度可以在风险敏感的应用中用于过滤低置信度的预测。然而，如果我们关注表1，很明显，这些方法通常导致故障预测性能在各种指标下变差。例如，在CIFAR-10/ResNet110上训练mixup和LS时，AURC（↓）分别增加了6.75和16.37个百分点。对于后处理校准技术，TS通过在保留的验证集上学习单个标量参数 $T$ 来校准概率。在我们的实验中，分别使用验证集和测试集学习参数 $T$ 表示为TS-valid和TS-optimal。表2显示，TS对故障预测几乎没有改善。这些结果是反直觉的，因为我们期望这些成功校准置信度的方法对故障预测有用。

#### 流行的OOD检测方法可能会损害故障预测
在实践中，用户自然期望好的置信度估计器可以帮助过滤来自未知类别的OOD样本和来自已知类别的错误分类的InD样本。然而，在表1中，我们观察到OOD检测方法通常导致故障预测性能在各种指标下变差。例如，在CIFAR-100/ResNet110上，最近提出的训练时OOD检测方法LogitNorm的AUROC（↑）为79.56%，比基线低5.35%。同样，这些后处理OOD检测方法，如ODIN、能量、ReAct和MLogit，也对故障预测任务产生负面影响。这意味着这些OOD检测方法使基于置信度排名区分错误预测和正确预测变得更困难。

#### 选择性风险分析
为了直观理解这些方法对故障预测的影响，图3绘制了风险覆盖曲线。具体而言，选择性风险是信任预测的经验损失或错误率，而覆盖率是未被拒绝预测的概率质量。直观上，一个更好的故障预测器在给定覆盖率下应该有较低的风险。从图3可以看出，与其他校准和OOD检测方法相比，基线具有最低的风险，这表明使用这些方法输出的置信度在做出决策时会不幸地增加风险。

#### 相同的观察结果推广到大规模数据集
在这里，我们验证了这些流行的校准和OOD检测方法通常在ImageNet数据集上损害故障预测，ImageNet包含1000个类别和超过120万张图像。我们训练了ResNet-18和ResNet-50，分别实现了70.20%和76.14%的top-1分类准确率。如图4所示，可以观察到类似的负面影响：那些广泛认可的校准和OOD检测方法导致故障预测性能比基线差。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/2f40d2c3302e49dca505c1cbd9eba0da.png)



### C. 进一步讨论和分析
1. 关于故障预测的校准讨论：适当的评分规则。为了更深入地了解，我们回顾了适当的评分规则，这是一个几十年前的概念，用于评估估计的分数 $S$ 如何解释观察到的标签 $Y$。最广泛使用的评分规则是对数损失： $\phi_{LL}(S, Y) := -\sum_{k=1}^K Y_k \log S_k$。请注意，评分规则适用于单个样本，对于数据集，使用所有样本的平均分数。使用规则 $\phi$ 的预期得分在估计分数向量 $S$ 上关于根据真实后验分布 $Q$ 抽取的类别标签 $Y$ 的情况下，给出 $s_\phi(S, Q) := \mathbb{E}_{Y \sim Q}[\phi(S, Y)]$。接下来，我们定义 $S$ 和 $Q$ 之间的散度为

$$
d_\phi(S, Q) := s_\phi(S, Q) - s_\phi(Q, Q)。
$$

如果一个评分规则的散度始终非负，并且 $d_\phi(S, Q) = 0$ 意味着 $S = Q$，则该评分规则被认为是适当的，如果进一步是严格适当的。举例来说，对数损失是一个严格适当的评分规则，而焦点损失则不是严格适当的。然后，我们提出评分规则分解如下：

**命题1（校准-区分分解）：** 令 $C$ 为联合校准分数，即 $C_k = P(Y = e_k \mid S = s)$ 对于 $k = 1, \ldots, K$。严格适当的评分规则的散度可以分解为

$$
\mathbb{E}[d_\phi(S, Y)] = \mathbb{E}[d_\phi(S, Q)] + \mathbb{E}[d_\phi(Q, Y)] = \mathbb{E}[d_\phi(S, C)] + \mathbb{E}[d_\phi(C, Q)] + \mathbb{E}[d_\phi(Q, Y)]，
$$

其中期望是在 $Y \sim Q$ 和 $X$ 上进行的。最后一项，称为不可减少损失（也称为固有损失），是由于在为样本分配确定性标签时存在的固有不确定性。认知损失是由于模型不是贝叶斯最优分类器。认知损失可以进一步分解为校准和分组损失。具体而言，校准损失衡量估计分数 $S$ 与具有相同校准分数 $C$ 的实例中的正例比例之间的差异；分组损失描述了具有相同置信度分数 $S$ 的许多实例具有不同的真实后验概率 $Q$。直观上，分组损失捕捉置信度分数区分样本的能力。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/76bdec07bbc441e5bd1b4fb6488752f5.png)




理想情况下，模型应该具有良好的校准和高区分能力。然而，区分和校准不一定同步，许多流行的方法忽略了区分部分。流行的校准方法，如mixup、LS、Focal和Lp范数，通常通过惩罚整个样本集的置信度到较低水平来改进校准。然而，这会导致不良效果：擦除样本难度的重要信息，这对于保持区分能力是有用的。从图5中也可以看出这一点：在训练期间正确分类样本的平均置信度明显降低，使得区分正确分类和错误分类的样本变得更加困难。此外，图5（右）中的置信度分布也显示了更好的校准可能导致更差的区分：根据分组损失的定义，我们估计置信度为0.95的实例密度，并用橙色（LS）和蓝色条（基线）标记结果。我们观察到，尽管LS导致了校准的置信度，但不幸的是，它增加了分组损失（较少的区分）。因此，根据置信度排名很难区分正确和错误的样本。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/971a175cb4354184a0bcf542ee778a17.png)




实际上，校准测量的是平均准确率和置信度之间的聚合不匹配，而没有考虑正确和错误预测之间的置信度分离。而仅有区分并不等于故障预测，因为具有高区分能力的分数估计器可能为错误分类的样本分配高置信度，而为正确分类的样本分配低置信度。因此，基于适当的评分分解，良好的校准和区分是准确概率估计的必要和充分特性。第四部分将显示我们的方法可以实现良好的故障预测和校准性能。

2. 关于故障预测的OOD检测讨论：在安全敏感的应用中，OOD样本和错误分类的InD样本都会导致显著的损失，因此应该被拒绝并交由人类处理。然而，如第三部分B所示，OOD检测方法通常使检测错误分类样本变得更加困难。为了进一步了解OOD检测方法对故障预测的负面影响，我们分别重新审视了贝叶斯最优分类器对故障预测和OOD检测的拒绝规则。

**命题2（故障预测的贝叶斯最优拒绝规则）：** 对于定义3中的故障预测风险 $R_{\text{fp}}(f, g)$，最小化 $R_{\text{fp}}(f, g)$ 的最优解 $g^*$ 由Chow规则给出

$$
g^*(x) = \mathbb{I}(\max_{y \in \mathcal{Y}} P(y \mid x) \geq 1 - c)，
$$

其中 $c \in (0, 1)$ 是拒绝成本。由于在实践中真实后验概率 $P(y \mid x)$ 是未知的，因此该规则不能直接使用。

**命题3（OOD检测的贝叶斯最优拒绝规则）：** 对于定义2中的OOD检测风险 $R_{\text{ood}}(g)$，最小化 $R_{\text{ood}}(g)$ 的最优解 $g^*$ 是

$$
g^*(x) = \mathbb{I}([\frac{p(x \mid \text{in})}{p(x \mid \text{out})}] > [\frac{1 - \pi_{\text{in}}}{\pi_{\text{in}}}])，
$$

其中 $\pi_{\text{in}} \in (0, 1)$ 是未知分布中InD和OOD数据的混合比率。



![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/9e522f36efa640c3959795c4b3dd59ba.png)



### 拒绝区域的不对齐
如命题2和3所示，故障预测的贝叶斯最优拒绝规则基于最大类别后验概率 $\max_{y \in \mathcal{Y}} P(y \mid x)$，而OOD检测拒绝样本基于小的密度比 $\frac{p(x \mid \text{in})}{p(x \mid \text{out})}$。图6展示了在 $\mathbb{R}$ 中两个高斯类别（具有相同方差）的情况下的拒绝区域示意图。具体而言，错误分类的InD样本位于不同类别之间的混淆区域，其中最大类别后验概率较低。基于贝叶斯最优拒绝规则，拒绝区域为 $\Omega_{\text{misInD}}$。至于OOD样本，它们在训练集中没有信息，可能是噪声或来自新类别的语义偏移示例。OOD样本的共同特征是它们远离已知类别的中心，如图6中的区域➀和➁所示。基于贝叶斯最优拒绝规则，拒绝区域 $\Omega_{\text{ood}}$ 位于两侧（➁）和原点周围（➀）。特别是，OOD拒绝区域➀与 $\Omega_{\text{misInD}}$ 重叠。因此，MSP分数拒绝两个类别之间的区域，可以作为错误分类和OOD检测的强大、通用的基线。

要检测那些位于➁的OOD样本，MSP无能为力，因为这些区域具有高MSP分数，如图6中红色虚线所示。这种情况正好与故障预测相反。命题3表明，InD和OOD的密度比是检测OOD样本的最优规则，这在图6中可以观察到。为此，许多流行的OOD检测方法显式或隐式地执行密度估计。例如，异常暴露（OE）与InD和OOD数据之间的二元区分相似，基于能量的OOD检测标准的贝叶斯最优解决方案等同于命题3中陈述的贝叶斯最优拒绝规则。然而，要区分InD和OOD，二元区分会压缩正确和错误InD样本的置信度分布，基于密度的规则如能量分数不适合检测错误分类样本。因此，OOD检测方法通常会损害故障预测。图7（在CIFAR-10上的ResNet110）证实了这些OOD检测方法导致错误分类的InD数据与正确分类的数据相比有更多重叠，即使用MSP置信度分数进行标准训练。

## IV. 寻找平坦极小值以实现可靠的置信度估计
如第三部分所报告的，那些流行的校准和OOD检测方法似乎没有比简单的基线更稳定地解决故障预测问题。在本节中，我们回答以下两个基本问题：（1）是否存在一种更有原则且不费力的方法来改进故障预测？（2）是否有可能同时改进校准、OOD检测和故障预测的性能？


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/93353b0ce3dd460b9830e8a663f1c740.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/3ac0f0784cf448ceadb67f353c0d5e80.png)


### A. 动机和方法论
1. **动机:** 平坦极小值与置信度分离之间的联系。正确和错误样本之间的置信度可分离性对故障预测至关重要。让我们考虑置信度可分离性如何影响正确样本的置信度鲁棒性：对于一个正确分类的样本，要变为错误分类，它必须降低在真实类别上的概率并增加在另一个（错误）类别上的概率。在此过程中，置信度边际起着关键作用：较大的置信度边际可以使更改预测类别标签变得更困难。有趣的是，模型的平坦性反映了在扰动模型权重时正确分类样本变为错误分类样本的敏感性。如图8所示，具有平坦极小值时，正确样本在权重扰动下难以错误分类，反之亦然。因此，我们推测平坦极小值的正确和错误样本之间的置信度差距大于尖锐极小值。

**表示学习和不确定性:** 高置信度的错误分类意味着样本被映射到错误类别的密度区域。这通常归因于样本和错误类别中出现的虚假相关。理论上证明，平坦极小值会导致不变和解缠表示，这对缓解虚假表示有效。因此，具有更少虚假或不相关表示，错误分类样本将接近决策边界，具有低置信度并且在错误类别中激活较少。此外，已经显示平坦极小值对应于参数空间中丰富的后验不确定性。因此，平坦极小值具有指示输入不确定性的优势。

**可靠过拟合现象:** 如图9所示，我们观察到一个有趣的现象，即在模型训练过程中，AUROC很容易过拟合。具体而言，测试准确率在最后阶段持续增加，而AUROC下降，使得故障预测变得困难。我们称这种现象为“可靠过拟合”，它存在于不同模型和数据集设置中，某种程度上类似于对抗鲁棒性文献中的鲁棒过拟合。由于平坦极小值已被验证在缓解鲁棒过拟合方面有效，我们期望它也能有利于故障预测。

2. **方法论:** 已经提出了几种寻找DNN平坦极小值的方法。我们选择随机权重平均（SWA）和敏锐感知最小化（SAM）作为两个代表性方法，因为它们的概念验证相对简单。具体而言，SWA简单地沿训练轨迹平均模型的多个参数，如下：

$$
\theta_t^{\text{SWA}} = \frac{\theta_{t-1}^{\text{SWA}} \times s + \theta_t}{s + 1}，
$$

其中 $s$ 表示要平均的过去检查点数量， $t$ 是训练周期， $\theta$ 是当前权重， $\theta^{\text{SWA}}$ 是平均权重。而SAM通过直接扰动权重找到平坦极小值。具体而言，令批次大小为 $m$，在批次数据上计算的损失为 $L = \frac{1}{n} \sum_{j=1}^n \ell_{\text{CE}}((x, y); \theta)$，其中 $\ell_{\text{CE}}$ 是交叉熵损失。然后，SAM的优化目标是

$$
\min_ \theta L(\theta + \epsilon^\star(\theta))， \quad \text{其中} \quad \epsilon^*(\theta) \approx \rho \frac{\nabla_\theta L(\theta)}{\|\nabla_\theta L(\theta)\|}。
$$

尽管SWA和SAM基于不同的机制找到平坦极小值，我们发现它们都改进了故障预测性能。这也激励我们将它们结合起来以获得更好的性能。我们将它们的结合称为FMFP（平坦极小值故障预测）。算法1给出了完整FMFP算法的伪代码，它是即插即用的，可以通过几行代码实现。

3. **理论分析:** 在第三部分C中，我们展示了故障预测和OOD检测的贝叶斯最优拒绝规则，以及拒绝区域的不对齐。不同于现有工作，专注于设计各种后处理分数（例如能量分数、最大对数分数、ConfidNet分数），一种有原则和根本的方法是学习贝叶斯类似分类器。在下文中，我们展示了贝叶斯分类器基于PAC贝叶斯框架更偏向平坦极小值。

PAC贝叶斯是理解许多机器学习算法泛化的通用框架。给定先验分布 $P$ 和分类器权重 $w$ 上的后验分布 $Q$，PAC贝叶斯框架根据后验和先验分布之间的KL散度，对分类器 $f_w$ 的期望误差进行泛化误差的约束。形式上，考虑后验分布 $Q$ 形式为 $w + v$，其中 $v$ 是随机变量。然后，我们有以下定理：

**定理1（PAC贝叶斯界）：** 对于任何 $\delta > 0$，在 $n$ 个训练样本的抽取上，至少有 $1 - \delta$ 的概率，分类器 $f_{w+v}$ 的期望误差可以约束为

$$
\mathbb{E}_v[L(f_{w+v})] \leq \mathbb{E}_v[L_{\text{train}}(f_{w+v})] + 4 \sqrt{\frac{1}{n} \left( \text{KL}(Q \| P) + \ln \frac{2n}{\delta} \right)}，
$$

其中 $\mathbb{E}_ v[L(f_ {w+v})]$ 是期望损失， $\mathbb{E}_ v[L_{\text{train}}(f_{w+v})]$ 是经验损失，它们的差异产生了泛化误差。根据文献，我们选择扰动 $v$ 为每个方向上方差为 $\sigma^2$ 的零均值球面高斯分布，并进一步设定 $\sigma = \alpha \|w\|$，这使得（13）中的第二项成为常数 $4 \sqrt{\frac{1}{n} \left( \frac{1}{2\alpha} + \ln \frac{2n}{\delta} \right)}$。然后，用 $L_{\text{train}}(f_w) + (\mathbb{E}_ v[L_{\text{train}}(f_{w+v})] - L_{\text{train}}(f_w))$ 替代 $\mathbb{E}_v[L_{\text{train}}(f_ {w+v})]$，分类器的期望误差可以约束（在训练数据上的 $1 - \delta$ 的概率）如下：

$$
\mathbb{E}_ v[L(f_{w+v})] \leq L_{\text{train}}(f_w) + \left( \mathbb{E}_ v[L_{\text{train}}(f_{w+v})] - L_{\text{train}}(f_w) \right) + 4 \sqrt{\frac{1}{n} \left( \frac{1}{2\alpha} + \ln \frac{2n}{\delta} \right)}，
$$

其中 $\left( \mathbb{E}_ v[L_{\text{train}}(f_{w+v})] - L_{\text{train}}(f_w) \right)$，即权重损失景观的平坦度的期望，约束了泛化误差。因此，平坦极小值技术优化了平坦度 $\left( \mathbb{E}_ v[L_{\text{train}}(f_{w+v})] - L_{\text{train}}(f_w) \right)$，以控制上述PAC贝叶斯界，导致泛化误差减少和更类似贝叶斯的分类器。

基于贝叶斯最优拒绝规则，通过平坦极小值学习的贝叶斯类似分类器可以改进故障预测和OOD检测的性能。具体而言，（1）最大类别后验拒绝规则（例如MSP分数）和（2）密度拒绝规则（例如能量分数）的性能都可以提高，这将在我们的实验中验证。

### B. 实验
实验设置。我们在CIFAR-10、CIFAR-100和Tiny-ImageNet上进行了各种网络架构的实验。对于比较方法，我们主要将我们的方法与基线和CRL进行比较，CRL是故障预测的最先进方法，优于代表性的贝叶斯方法。对于CRL，我们的实现基于官方开源代码。对于SWA和FMFP，按照文献中的建议使用周期性学习率调度。在CIFAR-10和CIFAR-100上的实验中，基线模型在120周期的检查点用作SWA和FMFP的初始点。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/34882e5f5471483abc48d84030f29c14.png)

### C. 平坦极小值改进了故障预测
比较结果总结在表III和表IV中。我们观察到基于平坦极小值的方法：SAM、SWA和FMFP（我们的）在各种指标上始终优于强基线、CRL和最先进的故障预测方法OpenMix。特别是FMFP通常产生最好的结果。例如，在ResNet110的情况下，我们的方法在CIFAR-10和CIFAR-100上分别具有3.94%和2.47%的更高AUROC值。

**可视化：** 在图10中，我们观察到基线模型的正确预测和错误预测严重重叠，使得区分它们变得困难。我们的方法显著将错误的置信度分布移到较小的值，并保持正确样本的置信度，这有助于根据置信度过滤错误分类。为了更具说明性，图11展示了一些在CIFAR-10上错误分类的样本及其对应的置信度分布。我们的方法在错误预测的类别上输出更低的置信度。此外，图12中的风险覆盖曲线显示，我们的方法在给定覆盖率下始终具有最低的风险。

**与ConfidNet及其变体的比较：** ConfidNet是一种利用训练集中的错误分类样本来训练辅助模型进行置信度估计的故障预测方法。SS通过陡坡损失提高了辅助模型的泛化能力。Qu等人利用元学习框架来训练辅助模型。这些方法的性能对训练集中错误分类实例的数量敏感。可以通过利用一个保留的验证集来训练辅助模型来缓解这个问题。在本文中，我们在CIFAR-10和CIFAR-100上使用VGG-16比较这些方法，遵循这些工作的设置。如表V所示，我们的方法始终优于ConfidNet、MCDropout和Trust-Score。此外，与ConfidNet的两阶段训练方法和MCDropout需要采样多次（例如100次）以获得贝叶斯推理不确定性相比，我们的方法更简单且更高效。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/5180f6c538de4e2e86260b00de348086.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/521fb8ceae5941af84976e734d1bb126.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/8dbe7f0789a04a628e8b3abfdf724fd2.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/5b2b69bab0464988bfa33ccc26a5b287.png)



**平坦极小值减轻了可靠过拟合：** 图13绘制了训练期间的AUROC曲线。尽管通过提前停止可以提高故障预测性能，但提前检查点的分类准确率要低得多。我们可以清楚地观察到，使用平坦极小值，可靠过拟合显著减少，AUROC曲线在训练结束时稳健提高。此外，平坦极小值还可以进一步提高分类准确率，避免在应用提前停止时故障预测和分类准确率之间的权衡。

**不同权重平均策略的比较：** 最近，权重平均成为获得平坦极小值的一种流行方式。例如，Wortsman等人提出了模型汤技术，平均微调的大预训练模型的权重。这里我们验证了均匀汤和贪心汤的有效性。不同于以前的工作，平均多个模型的权重，使用不同的超参数配置微调，我们在训练后期平均模型，即平均100个周期后的模型。如表VI所报告：（1）均匀和贪心权重平均策略对提高置信度估计有效。（2）均匀汤产生略低的准确性，但置信度估计性能比贪心策略更好。这是合理的，因为贪心策略强调准确性，而具有较高准确性的模型可能会遇到可靠过拟合问题，如图9所示。（3）我们的方法优于均匀和贪心策略。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/acf748abd446477c9c55f54ea4825ab1.png)



### D. 长尾识别中的故障预测
现有的置信度估计方法通常在平衡的训练集上进行评估。然而，现实世界中的类分布通常遵循长尾分布，其中头部类的训练样本远多于尾部类。例如，疾病诊断的类分布通常是长尾的，即正常样本多于疾病样本。在这种对故障敏感的应用中，可靠的置信度估计尤其重要。

**实验设置：** 我们使用两个流行的长尾分类数据集CIFAR-10-LT和CIFAR-100-LT，这些数据集是从原始CIFAR数据集中抽取的指数分布样本。遵循先前的工作，默认的不平衡比率 $\rho = 100$ 被使用，网络是ResNet-32。我们的实验基于先前工作的代码。

**长尾故障预测的挑战：** 我们首先在长尾设置中检查CRL的性能。具体而言，CRL已被证明在平衡数据集上是一种强大的故障预测方法。然而，如表VII所示，CRL通常在故障预测和分类准确率方面表现更差。此外，通过将CRL与最先进的长尾识别方法LA、CDT和VS结合，无法解决这一挑战。为什么CRL在长尾识别中失败？一个直观的解释是，模型未能很好地学习尾部类，因此更可能导致低置信度预测，如错误分类样本。

**主要结果：** 如表VII所示，我们的方法显著改善了那些长尾识别方法的故障预测。例如，在CIFAR-10-LT上使用VS，我们的方法实现了4.57%的平均FPR95（↓）降低和1.74%的平均AUROC（↑）提高，而不会降低原始分类准确率。最近，Li等人提出了一种基于证据的不确定性技术，名为TLC，优于许多其他长尾识别方法。在图14中，我们在相同的实验设置下将我们的方法（基于VS）与TLC进行比较。TLC和其他方法的结果来自文献。可以看出，我们的方法在故障预测性能方面始终更好。

#### 协变量偏移下的故障预测
现有的置信度估计工作主要考虑输入分布是静态的情况。然而，在现实世界的应用中，已部署的系统在非静止和不断变化的环境中运行，可能会编码受各种协变量偏移影响的输入。以自动驾驶系统为例：周围环境很容易改变，例如天气从晴天变为多云再到雨天。在这些领域变化条件下，模型仍然需要做出可靠的决策。因此，有必要评估在协变量偏移下置信度估计的性能。

**实验设置：** 模型在CIFAR-10/100上使用第四部分B中描述的默认训练设置进行训练，并在损坏的数据集CIFAR-10/100-C上进行评估。具体而言，损坏的数据集包含原始验证集的副本，具有15种类型的算法生成的损坏，包括噪声、模糊、天气和数字类别。每种类型的损坏有五个严重程度，共有75种不同的损坏。

**主要结果：** 图15显示了平均15种类型损坏的结果随着损坏严重程度的增加而变化。所有75种损坏的平均结果报告在表VIII中。可以看出，我们的方法始终优于基线和CRL。图16绘制了故障预测在最严重的五级损坏下的AUROC值。首先，我们可以观察到，与在干净的测试集上的结果相比，所有评估的方法在不同程度上表现下降。这表明在协变量偏移下的预测不太可靠。其次，我们的方法优于CRL，并显著提高了基线的性能。例如，对于在CIFAR-10上训练的模型，我们的方法在“失焦”损坏下的AUROC值高出13.05%。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/8f0ff043f46f476782ea1e3ce906492d.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/dbe0329778d448c895055b5ea54541ed.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e54274d516e046a997f48b76b6431816.png)


### E. 平坦极小值也改进了OOD检测
一个好的置信度估计器应该有助于区分OOD和错误分类的InD样本。因此，除了故障预测，我们还探索了提出的基于平坦极小值的方法的OOD检测能力。

**实验设置：** InD数据集为CIFAR-10和CIFAR-100。对于OOD数据集，我们遵循最近的工作，使用以下六个常见基准：Textures、SVHN、Place365、LSUN-C、LSUN-R和iSUN。在训练期间，模型只看到InD数据。在测试时，我们遇到InD和OOD数据的混合。例如，模型在训练期间只看到CIFAR-10的训练集；在测试时，遇到CIFAR-10测试集和一个OOD数据集的混合。我们使用标准指标来衡量OOD检测的质量：AUROC、AUPR和FPR95。补充材料提供了OOD数据集的详细信息和评估指标的定义。

**主要结果：** 我们报告了在六个OOD测试数据集上的平均OOD检测性能。结果显示，我们的方法在不同数据集和网络上可以达到最先进的性能。此外，由于我们基于平坦极小值的方法是一种训练时技术，它可以与任何其他后处理OOD检测方法（如能量和MLogit）结合，以获得更高的OOD检测性能。为了获得进一步的洞察，图17比较了基线和我们的方法在CIFAR-10上的ResNet110的softmax分数直方图分布。显然，我们的方法使分数在InD和OOD之间更加可区分，从而实现更有效的OOD检测。


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/84945faa45e641c1bdfe8e08dc65672a.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/552920d8252c4a66ab215bc9cc56817d.png)


## V. 结论
可靠的置信度估计可以使广泛的风险敏感领域受益，从医疗保健（例如临床决策）到交通（例如自动驾驶）再到商业应用。在本文中，通过严格的理解和广泛的实验，我们重新思考了校准和OOD检测方法在故障预测任务中的可靠性。例如，我们观察到简单的基线，即最大softmax概率分数，竟然可以优于现有方法检测分类故障。此外，我们将当前的评估扩展到更现实的设置，如长尾和分布偏移场景，并提出了一种基于平坦极小值的统一方法，产生了最先进的置信度估计性能。我们希望为机器学习研究人员提供对当前方法的更深入理解，并为机器学习从业者提供一个强大的基线，在实际应用中确保分类故障的安全性。
# 声明
本文内容为论文学习收获分享，受限于知识能力，本文队员问的理解可能存在偏差，最终内容以原论文为准。本文信息旨在传播和学术交流，其内容由作者负责，不代表本号观点。文中作品文字、图片等如涉及内容、版权和其他问题，请及时与我们联系，我们将在第一时间回复并处理。