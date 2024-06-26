# 题目：[A Flexible EM-Like Clustering Algorithm for Noisy Data](https://ieeexplore.ieee.org/document/10330026)  
## 一种灵活的EM类噪声数据聚类算法
**作者：Violeta Roizman; Matthieu Jonckheere; Frédéric Pascal** 

****

# 摘要
尽管非常流行，但众所周知，高斯混合模型（Gaussian mixture model）的期望最大化（Expectation-Maximization，EM）算法在处理非高斯分布或存在异常值或噪声时表现不佳。在本文中，我们提出了一种灵活的类EM聚类算法（Flexible EM-like Clustering Algorithm，FEMCA）：设计了一种新的聚类算法，遵循EM过程。它基于对聚类中心和协方差的估计。此外，使用半参数范式，该方法为每个数据点估计一个未知的尺度参数。这使得算法能够在不过分损失效率的情况下适应更重尾分布、噪声和异常值的各种经典场景。我们首先介绍了独立但不一定是相同分布的椭圆分布样本的一般底层模型。然后，我们在该背景下推导并分析了所提出的算法，特别是展示了底层数据分布的重要无分布特性。通过考虑第一种合成数据，分析了算法的收敛性和准确性特性。最后，我们展示了FEMCA在应用于真实数据集（如MNIST、NORB和20newsgroups）时，优于其他经典无监督方法，例如k-means、高斯混合模型的EM以及其最近的变体或谱聚类。
# 关键词
- 聚类 (Clustering)
- 高维数据 (high-dimensional data)
- 混合模型 (mixture models)
- 鲁棒估计 (robust estimation)
- 半参数模型 (semi-parametric model)

# I. 引言
聚类任务包括将一组元素根据同质属性/特征分组，这些属性/特征捕获了整个集合的某些重要结构。与其他无监督学习任务一样，由于近年来未标记数据量的显著增加，聚类引起了极大的兴趣。由于现实生活数据的特征在几何和统计术语上非常多样化，大量的研究工作致力于定义各种聚类算法，这些算法适应某些特定特征和结构属性，例如 [1]、[2]、[3]、[4]、[5]。我们引用 [6] 和 [7] 中的聚类综述，讨论了不同的方法以及如何根据设置选择方法。在不同类型的聚类算法中，用于估计潜在高斯混合模型（Gaussian Mixture Model，GMM）参数的期望最大化（Expectation-Maximization，EM）过程是一个非常流行的方法，因为其基于模型的特性通常允许在数据是低维的，并且聚类具有椭圆形状时，比其他算法有更好的性能。该模型将数据分布表示为由高斯分布混合的随机变量。相应的聚类准则很简单：所有来自给定正态分布的点都被认为属于同一个聚类。期望最大化算法（EM）[9]是一种通用的统计方法，用于基于似然最大化估计概率模型的参数。它是一个迭代算法，包含两个主要步骤：期望部分和最大化部分。特别是对于GMM情况，在最大化步骤中存在封闭形式的表达式以获得参数估计。

然而，其性能在各种对机器学习应用感兴趣的场景中显著下降：
- 当数据分布在尾部比高斯分布更重（或更轻）和/或存在异常值或噪声时（见，例如，[10]）。这种现象可以由算法计算的估计量的非鲁棒性来解释：均值和样本协方差矩阵 [11]。
-  数据中存在不同的尺度可能会使围绕它们最近的中心（例如，通过马氏距离）的观察值的全局排序变得复杂。对于协方差矩阵的估计，通常的归一化程序可能过于刚性，以至于在存在显著的类内和类间变异性时无法获得令人满意的聚类结果 [12]。
- 当维度增加时（即使在高斯情况下），协方差矩阵的估计受到高维性的严重影响，如 [13] 所示。这方面的一些解决方案包括正则化和节俭模型，这些模型通过限制协方差矩阵的形状以减少需要估计的参数数量 [14]。


为了在嘈杂和多样化数据的背景下提高GMM-EM聚类算法的性能，考虑了不同的策略。一种策略是修改模型以考虑噪声，另一种策略是保持原始模型并用能够处理异常值的其他估计量替换它们 [15]。已经开发了几种高斯混合模型的变体，以研究这一研究方向。特别是，一些变体针对更一般分布的混合问题，这使我们能够模拟更广泛的数据范围，并可能允许存在噪声和异常值。关于使用非高斯分布，[16] 提出了一个重要的模型，定义为多元t分布的混合。在这项工作中，作者建议了一种算法（在文献中称为t-EM或EMMIX）来估计具有已知和未知自由度的混合参数，通过最大化似然并解决聚类任务。最近，[17]、[18]、[19] 考虑了双曲和偏斜t分布。

其他值得一提的鲁棒聚类方法包括在通常的高斯似然中添加一个额外项的模型，以及受到常用鲁棒技术启发的算法，例如鲁棒点估计器、鲁棒尺度、观察值的权重和修剪技术。例如，[20] 考虑了存在作为背景的均匀噪声，而 [21] 提出了RIMLE，这是一种基于伪似然的算法，它过滤了低密度区域。[22] 用空间中位数和秩协方差矩阵（RCM）替换了通常的均值和样本协方差。[23] 引入了一个鲁棒尺度，以定义一个类似k均值的算法，该算法可以处理异常值。此外，[24] 提出了一种基于鲁棒泛函的鲁棒混合分布估计。此外，在 [25]、[26] 和 [27] 的工作中，提出了不同的观察值权重，其中小权重通常对应于在鲁棒文献中远离聚类中心的观察值。最后，修剪算法，如TCLUST [12]，为了在M步更好地估计参数，忽略了远离所有均值的数据点的比例。

此外，已经提出了超出EM框架的方法，源自监督学习算法中的提升方法。在 [28] 中，为了提高聚类性能，每个点都迭代地附加了一些权重。尽管这里采用了完全不同的方法，我们的目标也是定义一个通过每个数据点的额外参数带来的更大灵活性的算法。这种灵活性允许我们超越更传统的算法，并对一大类底层分布自适应。然而，与基于提升的方法不同，它间接地遵循了鲁棒统计方法的路径。

提出的方法受到椭圆对称（Elliptical Symmetric，ES）分布的鲁棒应用的启发 [29]、[30]。当然，椭圆分布在许多应用中得到了广泛的使用，在这些应用中，由于存在重尾或异常值，高斯分布无法逼近底层数据分布 [31]、[32]。这个大家族包括，除其他外，包含高斯、t−和k−分布的复合高斯分布 [33]、[34]、[35]，以及多变量广义高斯分布 [36]。
在本文中，我们提出了：
- 一个通用模型，涉及独立（但一般不是相同分布的）数据，每个数据点涉及一个尺度参数。
- 一个具有以下特点的聚类算法：1/ 它遵循EM算法的期望和最大化两个步骤；2/ 在E步，期望的条件对数似然导致与分布形状无关的估计量；3/ 在M步，它派生出聚类中心和协方差矩阵的估计，这些估计遵循经典的鲁棒估计量。

我们展示了，在温和的假设下，成员概率估计不依赖于数据分布的形状，使算法通用、简单和鲁棒。可以注意到，尺度/干扰参数的间接估计也可用于分类和异常值检测目的，通过区分数据并帮助数据分配 [37]，但这不是我们这里的范围。所提出算法的一个关键特性是自包含的，这意味着不需要调整额外的参数，就像上述方法（例如，惩罚参数、拒绝阈值和其他分布参数，如形状或自由度）一样。

在接下来的部分中，我们将包括实际和理论研究，提供算法性能的证据。特别是，我们使用各种论点从理论上证明了我们算法的效率：

1) 当底层模型属于椭圆分布类时，每个聚类具有不同的均值和散布矩阵，但聚类独立的密度生成器（即使在聚类内部也不同），那么成员概率的估计不依赖于每个特定的密度函数。这是因为这些概率估计不依赖于协方差矩阵的尺度因子，而只依赖于散布/散布矩阵。因此，只要满足这个假设，算法在未知密度生成器的情况下就不会犯匹配错误。这在命题4中显示。

2) 即使每个聚类的密度函数不同，也存在可以控制匹配误差的区域。我们使用具有各种自由度的t−分布给出了一个示例。见命题5。

从实际角度来看，与k-means、GMM的EM算法和HDBSCAN [38]、[39]、[40] 相比，当应用于真实数据集，如MNIST变体 [41]、NORB [42] 和20newsgroups [43] 时，所诱导的聚类性能得到了很大提高。与所提出的结果一致，先前对MNIST数据集分类的工作表明了聚类的非高斯性 [44]。与谱聚类和t-EM、TClust和RIMLE相比，我们的算法在经典案例中表现相似，在其他案例中要好得多。此外，所提出的算法即使在存在重尾分布或加性噪声的情况下，也能提供位置和散布参数的准确估计，正如在模拟中证明的，我们的算法击败了其他比较模型。
本文的其余部分组织如下。在第II节中，我们在详细介绍了感兴趣的模型之后，提出了聚类算法，并讨论了一些其重要方面，特别是通过考虑第一个合成数据，证明了参数估计的收敛性结果。第III节专门用于实验结果，这些结果使我们能够展示所提出方法在不同合成和真实数据集上与其他常用方法相比的改进性能。最后，在第IV节中，我们提出了结论和展望。

# II. 模型和理论依据

本节介绍了底层的理论模型和提出的聚类算法。设有 $n$ 个独立数据点的集合 $\{x_ i\}_  {i=1}^n$ ，它们属于 $K$ 个不同的类别 $C_ 1, \ldots, C_ K$ ，其中类别的基数为 $n_ i$ ，使得如果 $x_ i$ 属于类别 $C_ k$ ，则其分布由密度函数 $f_ {i,\theta_ k}$ 给出，该密度函数一方面依赖于干扰参数 $\tau_ {ik}$ ，另一方面依赖于类别参数 $\theta_ k$ 。请注意，由于这个干扰参数，点不是独立同分布的（i.i.d.），而只是独立的。按照聚类的惯例，我们通过未观测到的潜在变量（ $Z_ i$ ） $1 \leq i \leq n$ 来完善模型，这些变量表示每个观测值 $x_ i$ 的聚类标签，它们遵循具有参数 $\pi_ k$ 的多项分布。我们用 $\tilde{\theta}_ k = (\theta_ k, \pi_ k)$ 表示扩展的聚类参数。

模型的一般性水平。在一个特定的类别中，数据点共享相同的均值和散射矩阵。然而，数据点被假设为独立但不独立同分布的。实际上， $K$ 个类别仅由参数 $\theta_ k$ 表征，而分布的形状可以从一个观测值变化到另一个观测值。在接下来的段落中，我们将解释这样一个通用结构的相关性，其中我们为数据设定了一组分布。

我们考虑一个非常大的分布类别，以显著地推广传统的高斯混合模型：椭圆对称（ES）分布。我们将假设与类别 $k$ 中的点相关的密度函数遵循具有均值 $\mu_ k$ 和协方差矩阵 $\tau_ {ik}\Sigma_ k$ 的 ES 分布，并且可以写成：

$$
f_ {i,\theta_ k}(x_ i) = A_ {ik}|\Sigma_ k|^{-1/2}\tau^{-m/2}_ {ik} g_ {i,k}\left(\sqrt{\tau_ {ik}}(x_ i - \mu_ k)^T \Sigma^{-1}_ k (x_ i - \mu_ k)\right),
$$

其中 $A_ {ik}$ 是一个归一化常数， $g_ {i,k} : [0, \infty) \rightarrow [0, \infty)$ 是任何函数（称为密度生成器），使得（1）定义了一个概率密度函数（pdf）。矩阵 $\Sigma_ k$ 反映了 $x_ i$ 的协方差矩阵的结构。请注意，如果分布具有有限的二阶矩，则协方差矩阵等于 $\Sigma_ k$ 乘以一个比例因子（见[30]以了解更多细节）。这表示为 ES $(\mu_ k, \tau_ {ik}\Sigma_ k, g_ {i,k}(\cdot))$ 。请注意，聚类参数是 $\tilde{\theta}_  k = (\pi_ k, \mu_ k, \Sigma_ k)$ ，而干扰参数是 $\tau_ {ik}$ 。

在这个阶段，一些评论是必要的：

1. 当 $g_ {i,k} = g$ ， $\forall i, k$ ，则所有点都遵循相同的 ES 分布，但是每个类别具有不同的均值和协方差矩阵。然而，请注意，这个模型比高斯混合模型要通用得多，因为 ES 分布的类别要广泛得多，特别是，它包括比高斯分布更轻和更重的尾部。此外，干扰参数的存在使得模型通常是非独立同分布的。我们结果的一个重要方面是，我们的算法在这种情况下对函数 $g$ 不敏感，因此即使在未知函数 $g$ 的情况下，也能有效地处理实际数据集。

2. 当 $g_ {i,k} = g_ i$ ， $\forall i, k$ ，我们得到一个比前一个更一般的模型，其中，即使点的分布不依赖于类别，除了它们的均值和协方差之外，类别内的数据可能遵循例如具有固定均值和协方差矩阵的 ES 分布的混合（与类别无关）。这在实践中很重要，因为许多数据集是从具有不同特征的不同数据源编译的。在这种情况下，我们的算法仍然对函数 $g_ i$ 不敏感，提供了很多建模灵活性和不匹配鲁棒性。

3. 当 $g_ {i,k} = g_ k$ ， $\forall i, k$ ，那么我们正在考虑每个数据类别的一个不同的 ES 分布。对于这种设置，聚类结果确实依赖于 $g_ k$ ，这可能是获得合理结果的实际障碍。然而，我们展示了我们的方法可以减轻这种依赖性，在某些情况下导致良好的性能。

椭圆分布已经在许多应用中被使用，其中必须处理重尾或异常值的存在[31]，[32]。这个通用家族包括高斯、 $t$ -和 $k$ -分布等[33]，[34]，[35]。这个模型承认一个随机表示定理。如果向量 $x_ i \sim$ ES $(\mu_ k, \tau_ {ik}\Sigma_ k, g_ {i,k}(\cdot))$ ，则它如果且仅如果它具有以下随机表示[45]：

$$
x_ i \overset{d}{=} \mu_ k + \sqrt{\tau_ {ik}A_ k u_ i},
$$

其中非负实随机变量 $Q_ {ik}$ ，称为模量变量，独立于在单位 $m$-球上均匀分布的随机向量 $u_ i$ ，而 $A_ k^T A_ k$ 是 $\Sigma_ k$ 的一个分解，而 $\tau_ {ik}$ 是一个确定性但未知的干扰参数。

请注意，在这项工作中，我们假设 $C_ {ik} = \tau_ {ik} \Sigma_ k$ 也可以依赖于第 $i$ 个观测值，通过干扰参数。我们假设手头的分布具有二阶矩，并且为了可识别性目的， $C_ {ik}$ 是协方差矩阵。这个假设意味着 $Q_ {ik}$ 的特定归一化，即

$$
E[Q_ {ik}] = \text{rank}(C_ {ik}) (\text{即 rank}(\Sigma_ k)) = m,
$$

当 $\Sigma_ k$ 是满秩的，例如[46]。

最后，在散射矩阵 $\Sigma_ k$ 中仍然存在歧义。实际上，对于任何正实数 $c$ ， $(\tau_ {ik}, \Sigma_ k)$ 和 $(\tau_ {ik}/c, c \Sigma_ k)$ 导致相同的协方差矩阵 $C_ {ik}$ 。在这项工作中，我们选择将 $\Sigma_ k$ 的迹固定为 $m$ ，这意味着我们正式考虑形状矩阵 $\tilde{\Sigma}_ k = m \frac{\text{tr}(\Sigma_ k)}{\text{tr}(\Sigma_ k)} \Sigma_ k$ 作为聚类特征。由于对推导没有影响（见备注 3），我们为了简单起见保持 $\Sigma_ k$ 的符号。也可以选择其他归一化，例如，不改变聚类结果的情况下，对 $\Sigma_ k$ 施加单位行列式约束。

[46] 在复数情况下表明，给定从 $x_ i \sim$ CES $(0_ m, \tau_ {ik}\Sigma_ k, g_ {i,k}(\cdot))$ 的随机样本，使用最大似然估计（MLE）估计 $\tau_ {ik}$ 与 $\Sigma_ k$ 的估计是解耦的。此外，作者证明了 $\Sigma_ k$ 的最大似然估计量是 Tyler 估计量，无论函数 $g_ i$ 如何。这是一个重要的结果，突出了 Tyler 估计量在这类分布中的普适性。我们将基于 Tyler 估计量的这种分布自由属性构建我们的结果。

## A. M-步：混合模型的参数估计

与高斯混合模型（GMM）的期望最大化（EM）算法类似，我们在E步计算每个观测值和聚类的标签，而在M步，我们估计感兴趣的参数 $\theta = (\theta_ k)_ {k=1}^K$ 。

我们首先表达模型的期望条件对数似然。设 $x = (x_ 1, \ldots, x_ n)$ 为一个样本集， $Z = (Z_ 1, \ldots, Z_ n)$ 为相应的潜在变量， $\theta$ 为参数集。

$$
E_ {Z|x,\theta^\*}[l(Z, x; \theta)] = E_ {Z|x,\theta^\*} \left[ \sum_ {i=1}^n \sum_ {k=1}^K 1\{Z_ i=k\} |x_ i=xi \log (\pi_ k f_ {i,\theta_ k}(xi)) \right] = \sum_ {i=1}^n \sum_ {k=1}^K p_ {i,k}^{\theta^\*} \log(\pi_ k f_ {i,\theta_ k}(xi)),
$$

然后，替换密度函数，我们得到

$$
E_ {Z|x,\theta^\*} [l (Z, x; \theta)] = \sum_ {i=1}^n \sum_ {k=1}^K p_ {i,k}^{\theta^\*} \left[ \log(\pi_ k) + \log(A_ {ik}) + \log \left( \frac{\tau_ {ik}^{-m/2}}{|C_ {ik}|^{1/2}} g_ i,k \left( (xi - \mu_ k)^T C_ {ik}^{-1} (xi - \mu_ k) \right) \right) \right],
$$

其中 $p_ {i,k}^{\theta^\*} = P(Z_ i = k | x_ i = xi, \theta^\*)$ 与 $\sum_ {k=1}^K p_ {i,k}^{\theta^\*} = 1$ ，并且 $C_ {ik} = \tau_ {ik}\Sigma_ k$ 。条件概率 $p_ {i,k}^{\theta^\*}$ 是在E步中计算的。另一方面，参数 $\theta$ 在M步中通过最大似然方法导出。

我们现在给出两个命题，总结模型参数估计的推导。如前所述，一个关键步骤是将似然函数分解为两个因子，这进一步允许描述E步和M步中估计器的基本属性。在命题1中，我们推导了 $\tau$ 参数的估计器。然后在命题2中，我们推导了其余的模型参数。

**命题 1**：假设 $x_ 1, \ldots, x_ n$ 是独立样本，且 $x_ i \sim$ ES $(\mu_ k, \tau_ {ik}\Sigma_ k, g_ {i,k}(\cdot))$ 对某些 $k \in \{1, \ldots, K\}$ 。假设 $\int_ 0^\infty t^{m/2} g_ {i,k}(t) dt < \infty$ ，对于所有 $i, k$ 。那么， $\tau_ {ik}$ 参数的最大似然估计是与其他估计器解耦的。对于固定的参数 $\Sigma_ k$ 和 $\mu_ k$ ， $\tau_ {ik}$ 的估计器被计算为

$$
\hat{\tau}_ {ik} = (x_ i - \mu_ k)^T \Sigma_ k^{-1} (x_ i - \mu_ k) a_ {ik}, \quad \forall 1 \leq i \leq n, \forall 1 \leq k \leq K,
$$

其中 $a_ {ik} = \arg \sup_ {t} \{t^{m/2} g_ {i,k}(t)\}$ 。

**命题 2**：给定独立随机样本 $x_ 1, \ldots, x_ n$ ，潜在变量 $Z_ i$ ，以及模型的期望条件对数似然，对于 $k = 1, \ldots, K$ 的 $\theta_ k$ 的最大化导致估计器必须满足以下方程。闭合方程式为

$$
\hat{\pi}_ k = \frac{1}{n} \sum_ {i=1}^n p_ {i,k}^{\theta^\*},
$$

对于每个分布的比例，

$$
\hat{\mu}_ k = \frac{\sum_ {i=1}^n p_ {i,k}^{\theta^\*} x_ i}{\sum_ {i=1}^n p_ {i,k}^{\theta^\*} (x_ i - \hat{\mu}_ k)^T \Sigma_ k^{-1} (x_ i - \hat{\mu}_ k)},
$$

对于每个分布的均值，

$$
\hat{\Sigma}_ k = \frac{m}{n} \sum_ {i=1}^n w_ {i,k}^{\theta^\*} (x_ i - \hat{\mu}_ k) (x_ i - \hat{\mu}_ k)^T (x_ i - \hat{\mu}_ k)^T \Sigma_ k^{-1} (x_ i - \hat{\mu}_ k),
$$

其中 $w_ {i,k}^{\theta^\*} = p_ {i,k}^{\theta^\*} / \sum_ {l=1}^K p_ {i,l}^{\theta^\*}$ ，对于散射矩阵。

证明见附录B。

从命题2的推导中可以得出，存在一个由 $\mu_ k$ 和 $\Sigma_ k$ 的估计器组成的两个固定点方程系统。这个系统被迭代求解以获得这些耦合的估计器，如第II-D节所述。

我们现在可以证明算法的一个基本属性，即模型的似然单调性。我们稍后在第II-D节中通过模拟来说明这个属性。为了建立更精确的收敛保证，我们需要一种数据驱动的方法，如[48]中发展的方法。我们将这个分析留作未来的工作。

**命题 3**：给定（3）中的期望对数似然，观测似然

$$
l(x; \theta) = \sum_ {i=1}^n \log \sum_ {k=1}^K \pi_ k f_ {i,\theta_ k}(x_ i),
$$

并且假设与在命题2中导出的固定点方程系统相关的迭代收敛，由来自命题1和2的估计器更新定义的步骤导致一个序列 $\{\theta_ t\}_ {t=1}^N$ ，其似然逐渐增加。

证明见附录C。

重要的是要注意，在我们模型中估计器的推导结果导致均值和协方差矩阵的常规鲁棒估计器。具体来说，两者都可以被同化为具有特定 u 函数的 M-估计器[11]。实际上，均值和散射矩阵估计器的表达式非常接近 Tyler 的 M-估计器（见[49]，[50]以了解更多细节）。主要的区别来自于混合模型，导致不同分布的权重不同。然而，在类别概率相等的情况下，即 $p_ {i,k} = 1/K$ 对于 $k = 1, \ldots, K$ 和 $i = 1, \ldots, n$ ，我们完全得到了散射矩阵的 Tyler 的 M-估计器，而均值估计器仅在分母的平方根上有所不同（稍后解释）。尽管我们的估计器是作为常规的 MLE（但）针对参数化的（由于 $\tau_ {ik}$）椭圆分布导出的，但它们本质上是鲁棒的。实际上，如[51]中详细说明的，Tyler 的和 Maronna 的 M-估计器既可以通过特定模型的 MLE 方法获得（例如，Student-t M-估计器），也可以直接从其他代价函数获得（例如，Huber M-估计器），所有这些估计器根据定义都是鲁棒的。

因此，这种方法可以被视为 Tyler 的 M-估计器在混合情况下的推广。实际上，我们有对于 $\hat{\mu}_ k$

$$
\frac{1}{n} \sum_ {i=1}^n u_ 1 \left( (x_ i - \hat{\mu}_ k)^T \Sigma_ k^{-1} (x_ i - \hat{\mu}_ k) \right) (x_ i - \hat{\mu}_ k) = 0,
$$

其中 $u_ 1(t) = p_ {i,k}^{\theta^\*}/t$ ，而 $\hat{\Sigma}_ k$ 可以写成

$$
\hat{\Sigma}_ k = \frac{1}{n} \sum_ {i=1}^n u_ 2 \left( (x_ i - \hat{\mu}_ k)^T \Sigma_ k^{-1} (x_ i - \hat{\mu}_ k) \right) (x_ i - \hat{\mu}_ k) (x_ i - \hat{\mu}_ k)^T,
$$

其中 $u_ 2(t) = m w_ {i,k}^{\theta^\*}/t$ 并且 $w_ {i,k}^{\theta^\*} = \sum_ {l=1}^n p_ {i,l}^{\theta^\*} / n$ 。

这种与经典 Tyler 估计器的相似性解释了我们提议的鲁棒性。实际上，差异在于 uj(.) 函数中出现的 pik 和 wik 权重项，在鲁棒统计学文献中传统上引入。这些自然意味着这些 u1(.) 和 u2(.) 函数继续尊重 Tyler 的条件（尽管 [49] 使用了 u1(t^{1/2}) 而不是 u1(t)，见 [51] 以了解更多细节）。

固定点方程的收敛性在 [11] 中已经被证明，但在我们的案例中不满足对 u 函数的严格假设。另一方面，Kent 在 [52] 中证明了对于固定均值，协方差估计器的固定点方程在归一化约束下是收敛的。最后，他还表明，对于某些 u 函数，联合均值协方差估计归结为约束协方差估计。不幸的是，这个技巧在我们的情况下不起作用。因此，Tyler 估计器的固定点方程的联合收敛仍然是统计学中的一个开放问题，即使在没有混合的情况下。

我们稍后进行分析和模拟，证实了算法在实践中的鲁棒性。特别是，第III-A节中的设置包括具有重尾的分布、不同的分布和噪声。

## B. E-步：计算条件概率

与在命题2中推导的估计器不同，公式(5)表明 $\tau_ {ik}$ 参数的估计与表征相应椭圆对称分布的函数 $g_ {i,k}$ 相关。我们现在给出算法的一个核心结果。以下命题表明，当 $g_ {i,k} = g_ i$ 时，估计器 $p_ {i,k}$ 不依赖于密度生成器。

**命题 4**：给定独立随机样本 $x_ i \sim$ ES $(\mu_ k, \tau_ {ik}\Sigma_ k, g_ i(\cdot))$ 对某些 $k \in 1, ..., K$ ，得到的估计条件概率 $\hat{p}_ {i,k} = P_ {\theta_ k}(Z_ i = k | x_ i = x_ i)$ 对所有 $i = 1, ..., n$ 和 $k = 1, ..., K$ 具有以下表达式：

$$
\hat{p}_ {i,k} = \frac{\pi_ k |\Sigma_ k|^{-1/2} (x_ i - \hat{\mu}_ k)^T \hat{\Sigma}_ k^{-1} (x_ i - \hat{\mu}_ k)^{-m/2}}{\sum_ {j=1}^K \pi_ j |\Sigma_ j|^{-1/2} (x_ i - \hat{\mu}_ j)^T \hat{\Sigma}_ j^{-1} (x_ i - \hat{\mu}_ j)^{-m/2}},
$$

其中 $\hat{\pi}_ k, \hat{\mu}_ k$ 和 $\hat{\Sigma}_ k$ 如命题2中所述。

**证明**：见附录D。

**备注 3**：

- 命题4（结合命题2）的后果极为重要，因为它允许我们独立于 $g_ i$ 的分布和 $\tau_ {ik}$ 参数推导E步中所需的条件概率。换句话说，对于任何独立的具有均值 $\mu_ k$ 和协方差矩阵 $\tau_ {ik}\Sigma_ k$ 的ES分布观测值 $x_ i$ ，导出了一个唯一的EM算法，该算法不依赖于各种涉及的分布的形状。这非常重要，因为在现实生活应用中，对特定数据分布的精确知识缺乏是最常见的情况，而估计它可能会显著降低性能。

- 其次，它表明 $\Sigma_ k$ 估计器的归一化不影响E步中概率计算。换句话说，散射矩阵的归一化与聚类结果无关。另一方面， $\hat{\Sigma}_ k$ 的归一化仅影响 $\tau_ {ik}$ 参数的尺度。

- 特别地，当数据点来自一个ES分布的混合，即 $g = g_ i$ 对所有 $1 \leq i \leq n$ 时，包含在命题4的特殊情况中。我们注意到，当所有分布都是高斯分布时，这个特例中的一个特定示例被包括在内。如果 $x_ i \sim N(\mu_ k, \tau_ {ik}\Sigma_ k)$ ，则相应的密度生成器是 $g(t) = e^{-t/2}$ 。相应的最大化器是 $\arg \sup_ {t} \{t^{m/2}g(t)\} = m$ ，因此估计器如(5)中推导的那样是：

$$
\hat{\tau}_ {ik} = \frac{(x_ i - \hat{\mu}_ k)^T \hat{\Sigma}_ k^{-1} (x_ i - \hat{\mu}_ k)}{m}.
$$

- 当 $g_ {i,k} = g_ k$ 时，不能直接作为命题4的特殊情况处理。实际上，假设每个类别由一个共同的ES分布 $g_ k$ 绘制通常意味着额外的参数，例如，对于 $t$-分布的自由度 $\nu_ k$ ， $K$-分布和广义高斯分布的形状参数，依赖于 $k$ 。这些参数必须在M步中估计。我们在下一节中给出了一个特定于多元 $t$-分布混合的示例，其中有不同的自由度 $\nu_ k$ 。

### C. 不同类别的密度生成器

当密度生成器依赖于类别时，我们的计算表明 $p_ {i,k}$ 确实依赖于 $g_ k$ ，与前一种情况相反。当密度生成器已知（这在实践中相当不切实际），这种假设自然会提高聚类性能，因为向模型中添加了额外的先验信息。相反，当实际数据分布与假定的不一致时，它意味着性能损失。

为了说明在这种情况下达到的依赖类型，我们推导了多元 $t$-分布混合的特殊情况的E步，其中每个聚类有不同的 $g_ k$ 函数。即，有 $K$ 个不同的 $g_ k$ 函数，每个聚类一个。每个分布的概率密度函数由下式给出：

$$
f_ {i,\theta_ k}(x_ i) = \frac{\Gamma\left(\frac{\nu_ k + m}{2}\right)}{\Gamma\left(\frac{\nu_ k}{2}\right) |\Sigma_ k|^{1/2} (\sqrt{\nu_ k \pi \tau_ {ik}})^{-m}} \left(1 + (x_ i - \mu_ k)^T \Sigma_ k^{-1} (x_ i - \mu_ k) \frac{\tau_ {ik}}{\nu_ k}\right)^{-(\nu_ k + m)/2}.
$$

以下命题定量地说明了当 $\nu_ k$ 参数和维度 $m$ 以相同速率增长时，估计的条件概率与“高斯值”（即，对于类别独立的 $g_ i$ 获得的值）之间的关系。

**命题 5**：给定一个由 $K$ 个 $t$-分布混合的独立样本，其中 $x_ i \sim t_ {\nu_ k}$ ， $\nu_ k$ 是自由度。如果对于每个 $k$ ， $\nu_ k m \approx ck$ ，则

$$
\hat{p}_ {i,k} = \frac{\pi_ k L_ {0,ik}^{ck} / (1 + ck)}{\sum_ {j=1}^K \pi_ j L_ {0,ij}^{c_ j} / (1 + c_ j) + O\left(\frac{1}{m}\right)},
$$

其中 $L_ {0,ik} = (x_ i - \mu_ k)^T \Sigma_ k^{-1} (x_ i - \mu_ k)$ 。

**证明**：见附录E。

这个场景包括了所有 $\nu_ k$ 相等的情况（见备注3）。此外，当自由度很大，维度 $m$ 固定时，它还包括了高斯情况，如证明中详细说明的。最后，所有 $\nu$ 参数与维度 $m$ 不相上下的中间情况因此被证明与高斯计算非常接近。如果这两种条件都不适用，可以进行 $\nu_ k$ 的特定估计，并且必须在M步中执行。

## D. 实施细节和数值考虑

提出的算法的一般结构与经典的高斯混合模型（GMM）的EM算法相同。两种算法之间的差异在于 $\hat{p}_ {ij}$ 的表达式以及参数估计的递归更新方程。我们设计了M步的略有不同的变种，并研究了它们的收敛性、精度和速度。我们这样做是因为 $\mu$ 和 $\Sigma$ 的估计器是基于Tyler估计器的加权版本。更具体地说，基于方程（7）和（8），我们提出了四种加速算法以加快收敛速度。这些版本取决于两个不同的方面。一个方面是使用刚刚计算出的均值估计器，或者是循环中前一次迭代的估计器。另一个方面是提出根据Tyler估计器强调数据点的权重。第II节提到，位置和散射估计器与Tyler的估计器非常接近，直到Mahalanobis距离的平方根，当位置未知时。我们提出通过添加这个平方根来修改权重，以模仿Tyler的估计器。不同版本定义如下：

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/9dd00944155c4534aeb455bbbf0a3b51.png#pic_ center" width="70%" />
</div>

图1显示了在两种不同设置下，不同版本的算法用于估计 $\mu$ 和 $\Sigma$ 的固定点方程的收敛速度。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/27c49fd739e84f5cab9944c2d2666fc7.png#pic_ center" width="70%" />
</div>

我们分别研究了这两个参数，以查看变种是否以不同的方式影响每个收敛。第一个是一个简单的案例，其中有两个在维度 $m=10$ 中很好地分离的高斯分布（均值等于 $0_ m$ 和 $2*1_ m$ ，协方差矩阵是单位矩阵 $I_ m$ 和一个对角矩阵，其元素为0.25, 3.5, 0.25, 0.75, 1.5, 0.5, 1, 0.25, 1, 1）。第二个是一个具有重尾的两个 $t$-分布的混合，具有相同的参数（ $\nu_ 1 = \nu_ 2 = 3$）。如您所见，在两种情况下，所有算法版本都在固定点循环大约二十次迭代后达到收敛。图1显示版本3和4的收敛速度更快，如预期的那样。另一方面，我们在图2中研究了这两种不同情况下模型对数似然的演变。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/6a94043a225a41618c3f81e5ddbcb4e1.png#pic_ center" width="70%" />
</div>

在多元 $t$-分布的情况下，我们使用真实的自由度（ $\nu_ k = 3$）计算似然。该图显示了所有情况下似然的增加，并且版本1的模型收敛更快，因为即使其计算时间比版本3和4稍长，它更快地达到了正确的均值/散射估计器。版本1和2的估计器的准确性更好，因为它们是基于最大似然的（ML-based），而版本3和4则不是。基于这些图表和之前关于固定点快速收敛的研究（例如，见[53]），我们选择了版本1，因为它遵循了最初的提议，尽管对于固定点循环来说速度略慢于版本2，但对于算法的收敛来说速度更快。此外，我们将所有实验中的迭代次数固定为20。请注意，增加这个数字在聚类性能方面不会导致显著增加。

现在让我们讨论算法中使用的初始化和阈值。均值参数被初始化为 k-means 算法得到的均值。当 k-means 输出只有一个点的聚类时，我们重新运行 k-means，同时排除孤立的点。由于奇异性问题，我们将初始散射矩阵取为单位矩阵。我们将所有 $\tau$ 参数的初始值设置为1。对于收敛标志，我们考虑连续估计器的 $l_ 2$-范数差的 $10^{-6}$ 阈值，并将固定点循环的长度设置为20，基于前面的讨论。我们使用上述初始化对每次运行获得相同的最终聚类结果。在低维情况下，我们截断 $\tau$ 值以避免由于接近均值的点引起的数值问题。也就是说，如果 $\tau$ 小于 $10^{-12}$ ，我们将其值更改为所选的阈值。算法的 Python 实现在 https://github.com/violetr/frem 存储库中可用。

此外，重要的是要注意，在我们的方法中，对 $\hat{\Sigma}$ 的迹的约束（ $tr(\hat{\Sigma}) = m$）并不充当正则化程序，就像在 EM 类算法中通常的情况那样[12]，[21]。如备注3中提到的，迹约束不影响聚类结果。

最后，关于算法的复杂性，它与经典的高斯混合模型的 EM 算法的复杂性相同。E步的复杂性与通常的算法相同。对于 M 步，尽管包含了一个嵌套循环来解决固定点方程，但复杂性并没有增加，因为迭代次数是固定的，每次迭代的主要成本对应于与 GMM 的 EM 中一样的散射矩阵求逆。

# III. 实验结果

本节展示了使用合成数据和真实数据获得的实验结果。第III-A节说明了所提出的算法如何适应具有更重尾部分布和异常值的模拟数据，而第III-B节处理真实数据。我们在合成数据（我们知道真实参数值）的情况下研究了固定点方程的收敛性和估计误差。我们将结果与经典的高斯混合模型（GMM）的EM、多变量t分布的EM、TCLUST [12]和RIMLE [21]的结果进行了比较。对于真实数据，我们将聚类结果与k-means、HDBSCAN和谱聚类[54]的地面真实标签进行了比较。与前三种算法的比较是直接的，因为它们都有一个共同的主要参数（聚类的数量），我们在实验中固定并假设已知这个参数。关于实现，我们使用Scikit-learn [55]进行k-means和高斯混合模型，使用R包EMMIXskew [56]进行t分布的混合。对于TCLUST和RIMLE算法，我们设置聚类的数量，并使用其余参数的默认值。我们尽可能避免了TCLUST算法解决方案中的人工约束，这是由特征值约束阈值引起的。我们使用作者提供的R实现。对于谱聚类，我们运行了Scikit-learn实现，其中需要调整一个额外的参数以构建邻域图。我们将图中的邻居数设置为最大化轮廓系数[58]的数量。与HDBSCAN的公平比较更加困难，因为要调整的参数完全不同，而且比其它算法的参数不太直观。我们通过扫面选定值的网格选择了最佳轮廓系数对的参数。

然后我们使用聚类任务中已知的指标，即调整互信息（AMI）指数和调整兰德（AR）指数[59]来量化性能差异。在真实数据集的情况下，当将每个聚类标签与分类标签匹配时，我们还提供了正确分类率。在某些情况下，我们通过UMAP算法[61]获得的数据的2D嵌入来可视化不同聚类算法的结果标签，以更好地理解数据的性质和聚类结果。这个降维算法与t-SNE[62]具有相同的目标，但它的Python实现要快得多。

## A. 合成数据

为了比较不同算法的聚类性能，数据是根据不同分布、不同的 $\tau_ {ik}$ 值和不同参数模拟的。不同的设置在表I和II中报告。设置1和2是多变量t分布的混合。设置3是k分布、t分布和高斯分布的混合。在设置4中，我们在三个高斯分布中添加了均匀背景噪声，这个噪声占数据的10%。最后，设置5包括混合了两种分布的聚类。在这种情况下，我们混合了广义高斯分布（记为GN）、t分布和高斯分布（记为N）。在表II中，diag和diag†是具有迹m的对角矩阵，而diag*的迹为12。因此，设置1测试了算法在具有不同迹的散射矩阵存在时的性能。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/d4005a7b69584059b3b4e1d6142511f7.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/a8fc60d4e2214bd3a29c0dc020fc9dc5.png#pic_ center" width="70%" />
</div>


我们重复每个实验 $n_ {rep} = 200$ 次，并收集估计误差的平均值和标准差。对于矩阵，我们计算真实散射矩阵参数与其估计值之间差异的Frobenius范数，除以矩阵大小。为了公平比较估计性能，我们考虑了 $\Sigma$ 的估计值，直到一个常数。换句话说，我们将所有 $\Sigma$ 的估计器归一化，使其具有正确的迹。报告的估计误差是按以下方式计算的：

$$
\frac{1}{m^2} \sum_ {l=1}^m \sum_ {o=1}^m \left( (\Sigma_ k)_ {lo} - \frac{\hat{\Sigma}_ k \cdot tr(\Sigma_ k)}{tr(\hat{\Sigma}_ k)} \right)^2
$$

当估计 $\mu$ 时，计算误差的 $l_ 2$ 范数。  $\pi_ k$ 向量，对应于分布比例，是从避免产生平凡和巨大聚类的一组可能性中随机选择的。这些情况由于它所暗示的病态聚类问题而被避免。

在这些实验中，我们包括了所有估计参数的算法，因此我们没有包括k-means、谱聚类和HDBSCAN的比较。表III显示了所有设置中模型主要参数估计误差的平均值。此外，我们在表IV中报告了聚类指标。图3的箱线图以视觉方式总结了这些度量的分布。在大多数情况下，EM for GMM (GMM-EM)方法的结果很差且方差很大。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/5f3af514f3ba49608367c2c03bac377b.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/474df6d3d453498fa62c9fde2ca08029.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/bd736d882ec64bfdb648f2a68522c2e1.png#pic_ center" width="70%" />
</div>

在设置1和2中，分布是多变量t-Student分布，它们之间的唯一区别在于自由度。在这些设置中，所提出的算法被称为灵活EM算法（FEMCA），并且t-EM的误差值小于GMM-EM值。这种预测性能的提高可以用重尾分布或存在异常值时估计器的鲁棒性来解释。值得注意的是，在设置2中，所考虑的分布具有更大的自由度（尾部更轻），GMM-EM的性能比设置1要好得多。然而，虽然TCCLUST和RIMLE在设置1中的性能相似，但在设置2中RIMLE有很大的方差和更差的估计。这种现象是由于将点过度估计为噪声/异常值。另一方面，FEMCA和t-EM在两种设置中的表现非常相似，FEMCA在Σ估计中略有改进。如表所示，我们的鲁棒算法在平均性能上即使在t分布的情况下也表现得很好，其中t-EM算法完全适应。我们的方法在设计上表现出非常稳定的表现，在广泛的案例中表现良好。当然，当数据完全遵循特定模型，例如e.g., a mixture of t-distributions，最好的算法将是ML-based one (in this case, the t-EM algorithm)。然而，FEMCA也几乎表现得和t-EM一样好。但在各种其他场景中（数据来自不同的模型，数据中存在异常值），FEMCA将明显优于传统模型基础算法，这些算法不是自适应的。

在设置3中，我们混合了三种不同的分布（k分布、t分布和高斯分布），FEMCA在大多数情况下都优于其他算法。图3显示，在少数情况下FEMCA的性能不佳。值得注意的是，用于推导FEMCA的模型假设，即未知的 $\tau_ {ik}$ 和每个观测值的不同分布，非常通用。它允许成功处理不同分布的混合，而不增加计算成本，这似乎是这项工作的一个重要贡献。

在设置4中，我们在三个高斯分布中添加了均匀背景噪声，这个噪声占数据的10%。在这种情况下，TCCLUST和RIMLE的性能最好，这似乎是合理的，因为它们的设计与数据生成过程相匹配。紧随其后的是FEMCA，考虑到我们没有拒绝异常值，因此那些本质上被错误分类。当我们为度量计算排除噪声时，这三个算法的分类性能同样好。然而，TCCLUST算法是使用真实的异常值比例计算的。此外，对于FEMCA、RIMLE和TCCLUST，参数估计同样好。

在设置5中，FEMCA和RIMLE与其它算法相比表现非常好。GMM-EM的性能非常差，因为它无法处理来自重尾的异常值。将两种分布组合在一个聚类中对于t-EM和TCCLUST来说很难拟合。模型对于t-EM来说太通用了，TCCLUST可能因为噪声率不足而无法避免重尾。

为了总结，所提出的算法在设计上表现出非常稳定的表现，在广泛的案例中表现良好。实际上，当数据完全遵循特定模型，例如混合 t-分布时，最好的算法将是最大似然估计（MLE）基础的算法（在这种情况下是 t-EM 算法）。然而，FEMCA 也几乎表现得和 t-EM 一样好。但在各种其他场景中（数据来自不同的模型，数据中存在异常值），FEMCA 将明显优于传统模型基础算法，这些算法不是自适应的。

## B. 真实数据

提出的 FEMCA 算法已在三个不同的真实数据集上进行了测试：MNIST [41]、小型 NORB [42] 和 20newsgroup [43]。MNIST 手写数字数据集已成为分类/聚类方法的标准基准。我们对相似对的平衡子集（3-8 和 1-7）以及数字集（3-8-6）应用 FEMCA 以发现组。我们还在后一个子集上人为地添加了少量噪声，通过随机添加一些剩余的不同数字。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/56753439cf3c4884ab5c8544d6ee1941.png#pic_ center" width="70%" />
</div>

如文献中的许多应用示例一样，我们首先应用 PCA 以使用一些有意义的特征而不是原始数据 [62]。我们在解释的方差和维数的诅咒效应之间做出折衷。降维数据的维度显示在表 V 之下的列 m 中。由于算法的随机性质，我们多次运行每个算法（nrep = 50），并报告指标的中值。FEMCA 的指标几乎总是相同的，这解释了为什么我们不报告方差。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/610633812f5d4819bb5335444c0be0fe.png#pic_ center" width="70%" />
</div>

如表 VI、VII 和 VIII 所示，在大多数情况下，我们获得的所有指标的值都优于其他分区技术产生的值。这可以通过增加灵活性和减少异常值对估计过程的影响来解释。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/53c303dd6e67462c99733ddec4c43efc.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/bdf3330bbdbe45cd83ad0e175f0de040.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/19a2b201de6e46299d86ae8ff491aa59.png#pic_ center" width="70%" />
</div>

更具体地说，FEMCA 在以下情况下不提供最佳结果：

- MNIST 7-1 场景的 AMI 和 AR 指标，其中 t-EM 表现最佳。
- MNIST 3-8-6 及其噪声变种的三种标准，其中谱聚类和 TCLUST 分别表现更好。

FEMCA 性能下降的情况在大多数情况下都在 1% 左右或更少，突出了该方法的鲁棒性：“在大多数情况下比现有方法更好或明显更好，在其他情况下可比”。此外，这些场景总是对应于更简单的场景，没有噪声且具有良好分离的聚类或完全设计用于最佳算法（MNIST 3-6-8 加噪声对于 TCLUST）。

我们收集了 HDBSCAN 算法的聚类结果，该算法使用了其两个主要参数的网格值。与地面真实标签比较的所有计算出的度量结果都很差，接近于 0。我们在图 7 中展示了 3-8 MNIST 子集的最佳聚类结果，其中算法将许多数据点分类为噪声。如果仅在非噪声标记的数据点上计算度量，则聚类几乎是完美的。这种行为可能是由于数据的维度，对于 HDBSCAN 来说似乎太高了。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/001b7c852bbb4afcb95b51a6053e5c73.png#pic_ center" width="70%" />
</div>

此外，我们测试了降维技术 UMAP 和 t-SNE 在聚类任务之前。在仔细调整参数后，所有指标都有所改进。在这种情况下，所提出的方法与经典的 GMM-EM 表现相似，因为这些嵌入方法倾向于将异常值和噪声吸引到聚类中。然而，这些非线性可视化方法不推荐用于聚类前提取特征，因为根据参数选择可能会出现虚构效果。

对于 NORB 数据集（图 5 显示了一些代表性样本），k-means、GMM-EM、谱聚类和 UMAP+HDBSCAN 的表现并不令人满意，因为它们最终将亮度作为主要的分类方面。相比之下，t-EM 和 FEMCA 的性能远远超过了它们，如表 VI、VII 和 VIII 所示。通过图 6 中基于不同方法产生的分类的二维嵌入，可以强调这一点，其中标签颜色的 UMAP 嵌入显示了数据。

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/d88cbda017e64b52b33f26e6c463174e.png#pic_ center" width="70%" />
</div>

<div align=center>
  <img src="https://img-blog.csdnimg.cn/direct/5ce04753428047ad8f4eb8452f7ded22.png#pic_ center" width="70%" />
</div>

最后，20newsgroup 数据集是从新闻语料库中构建的词袋模型。每篇新闻都通过主题建模被分类到二十个组中。再一次，我们在应用 PCA 后将我们的方法的性能与 k-means、EM、t-EM、TCCLUST、RIMLE 和谱聚类算法进行了比较。相应的结果也呈现在表 VI、VII 和 VIII 中。可以看出，k-means、TCCLUST、RIMLE 和谱聚类的表现很差，而 GMM-EM 和 t-EM 的性能优于它们。然而，所提出的 FEMCA 的结果远优于其他算法。不清楚为什么谱聚类在这个数据集上表现如此差；可能是由于聚类之间缺乏分离和/或存在破坏性能的噪声。最后，RIMLE 算法在这个数据集上的非常差的能力可以解释为参数选择过高估计了噪声。

# IV. 结论

在本文中，我们提出了一种鲁棒的聚类算法，它在合成数据和真实多样化数据上的表现优于多种现有的最先进算法。其优势源于对数据分布的一般模型，其中每个数据点都是由其自己的椭圆对称分布生成的。这个提议的良好理论属性已经通过模拟得到研究和支持。这种模型的灵活性使其特别适合于分析具有重尾分布和/或噪声污染的数据。有趣的是，在对数据的温和假设下，估计的成员概率不依赖于数据分布，使算法更简单（不需要在每一步重新估计似然），灵活且鲁棒。此外，为每个数据点估计一个尺度参数的原始方法使算法在相对高维设置中具有竞争力。在模拟数据上，我们获得了准确的估计和良好的分类率。当然，与数据分布完全一致的模型是最好的模型，例如，当混合实际上是高斯时，GMM-EM 优于所有其他方法，但只是边缘优势。我们的方法在所有考虑的场景中都表现良好。对于我们考虑的真实数据集，我们已经展示了所提出的方法比 k-means、GMM-EM 和 t-EM 提供了更好的结果。它还与谱聚类、TCCLUST 和 RIMLE 竞争，并且在 HDBSCAN 和谱聚类完全失效的情况下仍然提供非常好的结果。
