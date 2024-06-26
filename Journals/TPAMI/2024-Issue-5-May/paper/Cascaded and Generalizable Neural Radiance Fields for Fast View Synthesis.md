# 题目：[Cascaded and Generalizable Neural Radiance Fields for Fast View Synthesis](https://ieeexplore.ieee.org/document/10328666/)
## 用于快速视图合成的级联和通用化的神经辐射场
**作者：Phong Nguyen-Ha, Lam Huynh, Esa Rahtu, Jiri Matas, and Janne Heikkilä**
****
# 摘要
我们提出了一种级联和可泛化的神经辐射场方法，用于视图合成。最近的泛化视图合成方法可以使用一组附近的输入视图渲染高质量的新视图。然而，由于神经辐射场的均匀点采样特性，渲染速度仍然很慢。现有的特定场景方法可以有效地训练和渲染新视图，但不能泛化到未见过的数据。我们的方法通过提出两个新颖的模块来解决快速和泛化视图合成的问题：一个粗略的辐射场预测器和一个基于卷积的神经渲染器。该架构基于隐式神经场推断一致的场景几何，并使用单个GPU高效渲染新视图。我们首先在DTU数据集的多个3D场景上训练CG-NeRF，网络仅使用光度损失就能在未见过的实拍和合成数据上产生高质量和准确的新视图。此外，我们的方法可以利用单个场景的更密集的参考图像集来产生准确的新视图，而不依赖于额外的显式表示，同时保持预训练模型的高速渲染。实验结果表明，CG-NeRF在各种合成和真实数据集上超越了最先进的可泛化神经渲染方法。

# 关键词
- 神经渲染
- 新视图合成
- 基于体积的渲染

# I. 引言
新视图合成（NVS）是计算机视觉和计算机图形学中一个长期存在的任务，它在自由视角视频、远程存在和混合现实中有应用[1]。新视图合成是一个挑战性的问题，因为它需要从一组稀疏的参考视图捕获视觉内容，并为未见过的的目标视图进行合成。这个问题之所以具有挑战性，是因为视图之间的映射取决于场景的3D几何结构以及视图之间的相机姿态。此外，NVS不仅需要在视图之间传播信息，还需要对目标视图中由于遮挡或有限视野而不可见的细节进行幻觉。

早期的NVS方法通过在射线[2]或像素空间[3]中进行插值来生成目标视图。随后的工作利用了如极线一致性[4]这样的几何约束，以实现输入视图的深度感知变形。这些基于插值的方法由于遮挡和不准确的几何形状而产生了伪影。后来的工作试图通过将深度值传播到相似的像素[5]或通过软3D重建[6]来修补这些伪影。然而，这些方法无法利用深度信息来细化合成图像或处理时间不一致的不可避免问题。最近，神经辐射场（NeRF）通过隐式表示场景的3D结构并渲染新的逼真图像，对NVS研究产生了显著影响。NeRF的主要缺点有两个：i) 需要为每个新场景单独从头开始训练；ii) 渲染速度慢。此外，NeRF的每场景优化过程既耗时又需要为每个场景密集捕获图像。

最近的方法[8]、[9]、[10]、[11]、[12]、[13]通过训练一个泛化的NeRF模型来解决前者的问题，以适应未见过的场景。标准策略是从附近视图的源图像中提取特征，并对NeRF渲染器进行条件化。尽管这些模型能够泛化到新场景，但渲染速度是一个瓶颈，它们无法以交互速率渲染新视图。Chen等人[10]通过使用耗时的3D卷积和多层感知器（MLP）网络将多视图输入特征解码为高分辨率目标图像的体密度和辐射颜色。渲染此类图像需要向模型查询数百万个输入3D点，因此，在单次前向传递中渲染整个新视图并非易事。最近还有一些特定于场景的基于NeRF的方法，可以在实时渲染逼真的新图像，并且训练时间不到一个小时。尽管取得了令人印象深刻的结果，但这些方法通常依赖于可微的显式体素表示[14]、[15]、[16]或多分辨率哈希表[17]来存储神经场景表示。因此，这些方法需要一个完全新的每场景优化步骤来渲染未见数据的新视图。

本工作通过提出一种新颖高效的视图合成流水线来解决上述问题，该流水线在训练和测试期间可以在单次前向传递中渲染整个视图。受最近提出的作品[18]的启发，我们采用了粗到细的RGB和深度渲染方案来加速渲染过程。类似于MVSNeRF[10]，我们还通过一个浅层但高效的基于注意力的网络从一些非结构化的多视图输入图像中推断出低分辨率的3D体积。我们发现，由于采样的3D点数量减少，使用NeRF合成低分辨率的新视图既快速又高效。此外，NeRF的体积渲染在新视点提供了低分辨率的辐射特征和深度图。与使用耗时的粗到细渲染方法[7]、[8]不同，我们使用推断出的深度图来产生目标视点的近似深度特征，然后将它们与辐射特征融合作为输入到基于卷积的神经渲染器。然后我们训练这两个网络从低分辨率辐射特征中合成高分辨率的目标图像。渲染整个新视图还允许我们使用感知损失[19]或对抗性训练[20]，提高生成图像的整体质量。我们还包括了一个正则化损失，以确保预测的最终图像与粗略辐射场预测器估计的粗略新视图一致。
我们训练的CG-NeRF模型在接近输入视点的目标姿态上呈现合理的结果。然而，当我们从附近的源视图进一步外推目标时，性能会下降。以前的工作[10]、[14]、[15]、[16]通过使用更密集的输入图像集来学习辐射场的混合隐式-显式表示，这些图像覆盖了单个场景的更多视图。类似地，对预训练的CGNeRF模型进行10-15分钟的微调，与特定场景方法[13]、[16]相比，可以产生与前者相当的最先进的结果。预训练和微调的CG-NeRF模型都不依赖于显式数据结构，而是依赖于最接近目标的少数选定参考视图。正如图1所示，我们还观察到CG-NeRF模型生成的新视图在视觉质量方面与其他视图合成方法[8]、[10]、[21]相比有明显改进。请注意，我们的方法不依赖于深度监督[18]来提高合成图像的质量。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/57366c964a734cf49d934fa6c1fc5f08.png" width="70%" /> </div>


CG-NeRF通过轻量级视图合成网络显示出强大的泛化能力，可以渲染新视角下逼真的图像。如果用额外的图像进一步优化，CG-NeRF将超越最近提出的可泛化视图合成方法[10]、[13]、[21]和每场景优化模型[7]、[16]。本工作的主要贡献包括：
- 一个高效的稀疏视图合成网络，采用粗略的辐射场预测器和神经渲染器，比NeRF[7]及其变体[10]、[13]、[21]快大约两个数量级的速度有效预测新图像。
- 提出的场景特定模型仅需要使用更多图像对预训练模型进行10-15分钟的微调。此外，CG-NeRF不需要额外的深度监督。
- CG-NeRF在新视图合成方面取得了最先进的结果，无论是在真实还是合成数据集上，如DTU[22]、Synthetic-NeRF[7]、Forward-Facing[23]和Tank&Temples[24]。

我们将在论文发表后公开发布源代码和神经网络模型。

# III. 提出的方法
本节详细描述了CG-NeRF的架构，它由两个模块组成：一个粗略辐射场预测器（第III-A节），它以较低的分辨率生成场景的几何和外观；以及一个基于卷积的神经渲染器（第III-B节），它结合了粗略和精细的特征来生成原始尺寸的最终目标图像。此外，我们还讨论了用于训练可泛化的CG-NeRF模型的损失函数，然后对单个场景进行微调（第III-C节）。

## A. 粗略辐射场预测器
我们对粗略辐射场的推断方法与最近关于泛化视图合成的许多工作是正交的。尽管这些方法的结果令人印象深刻，但它们无法实现快速视图合成。由于每个像素都是独立渲染的，数百万个查询的3D点必须通过深度网络。这种方案是昂贵的，因为查询的数量远大于渲染的像素总数。

CG-NeRF与上述方法的主要区别在于，我们在较低的分辨率下推断辐射场，以减少查询输入并加快渲染过程。通过这样做，我们还可以在单个前向传递中获得整个目标视图的几何和外观特征。我们的方法通过避免将所有查询的3D点分割成多个小块，并将每个小块作为小图像块进行渲染，从而绕过了NeRF的缓慢渲染。因此，NeRF需要多个前向传递来渲染新视图的补丁。基于补丁的渲染策略还阻止了NeRF及其变体使用GAN或感知损失来训练他们的模型，如果在训练期间生成了随机像素。相比之下，我们可以在这些损失之间训练CG-NeRF，即地面真实图像和估计的新视图在低分辨率和高分辨率之间的损失（见第III-C节）。

**特征提取：** 我们首先描述我们的流程（见图2），用于估计给定一组N个非结构化输入图像{In}及其姿态的目标视点的粗略辐射场。每个输入图像In首先被送入特征金字塔网络，以提取Fc∈ RH/4×W/4×C 和 Ff∈RH×W×C/4，分别是粗略和精细的2D图像特征。注意，Ff具有与原始输入相同的高度H和宽度W，因此我们稍后可以使用它们进行粗略到精细的合成。我们在粗略层次上不知道场景几何形状，因此我们均匀地采样了几个K个虚拟深度平面。因此，我们利用每个输入视图的粗略特征来构建目标视点的成本体积。这些特征Fc通过双线性采样变形到多个假设深度平面。然后，变形后的特征被连接起来构建每个视图的粗略体积Vn。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/664358a2706d4f4da3786bf10d1854dd.png" width="70%" /> </div>


**多视图注意力学习：** 每个体积Vn包含目标视图的多个平面特征，因此需要一个空间推理架构来聚合这些N个体积，然后再进行神经渲染步骤。以前的工作使用普通的Transformers，由于多头注意力的大量计算，导致新视图的推理速度变慢。基于MVS的方法选择基于均值和方差的体积，稍后3D UNet可以处理以推断统一的场景编码体积。然而，由于与基于注意力的架构相比，3D Unet的接受场较小，因此受到限制。

在这项工作中，我们通过使用单个MobileViT块，这是Transformers的更节省内存的变体，结合了两种方法的优点。我们还计算了N个体积Vn之间的均值和方差，并将它们作为统计体积连接起来，然后传递给单个MobileViT块。该块通过多头注意力学习非重叠补丁之间的长期依赖关系。我们配置MobileViT块的输入和输出通道，产生一个统一的体积ˆV，其空间维度与Vn相同。通过学习关注提取的多视图特征，推断出的体积编码了场景几何和外观，这些可以稍后处理成体积密度和视图依赖特征，用于视图合成。

**粗略辐射场：** 使用统一的粗略体积ˆV，我们的方法学习了一个MLP网络M（见图3左）来回归体积密度σk和3D点xk及其观察方向dk的外观特征fk。具体来说，每个3D点xk是来自目标相机的射线与虚拟深度平面的交点。我们通过三线性插值获得采样点xk的特征ˆV(xk)。然后，粗略辐射场如下计算：

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/d6eedaa5889c4f8aaf57ac085caa05cb.png" width="70%" /> </div>


$$
(\sigma_k, f_k) = M\left[
\gamma(x_k), \gamma(d_k), \hat{V}(x_k)
\right]
\tag{1}
$$

其中γ是位置编码函数。在这项工作中，我们估计每点特征fk，这些特征稍后可以用于神经渲染器（在第III-B节中描述）。

我们设计模型M作为一个浅层MLP网络，以便训练/推理速度可以比NeRF的对应物更快。我们不是直接将位置嵌入γ(xk)和γ(d_k)连接到图像特征上，而是使用两个不同的全连接层，并将两个嵌入投影到潜在空间中，然后再与插值特征ˆV(xk)组合。增加这两个额外的层不会增加推理时间，但进一步提高了模型的学习能力。此外，由于将整个网络作为单个GPU内核处理，我们提出的架构比NeRF的对应物运行得更高效。

我们还采用[7]的体积渲染方法，通过可微分的光线步进来渲染新深度和特征。具体来说，我们可以通过沿射线r累积K个采样三维点的fk和σk来计算每射线特征Fr如下：

$$
\begin{align}
F_r = \sum_{k=1}^{K} \tau_k (1 - \exp(-\sigma_k \Delta k)) f_k \tag{2}
\end{align}
$$

$$
\begin{align}
\tau_k = \exp\left[
-\sum_{t=1}^{k-1} \sigma_t \Delta t
\right] \tag{3}
\end{align}
$$

其中 τi 是从射线原点到点 xk 的累积体积透射率，Δk 是相邻采样点之间的距离。收集目标相机的所有射线特征 Fr，我们获得了新视图的粗略特征图 F ∈ RH/4×W/4×C。我们还可以通过计算估计密度和 xk 的深度值的加权和，作为体积渲染的副产品，获得时间上一致的深度图 D ∈ RH/4×W/4。在没有学习预测原始分辨率下的新深度图的情况下，我们上采样粗略深度图 D，并使用它们通过特征融合步骤结合高分辨率输入特征 Ff_n。

## B. 神经渲染器

**特征融合：** 一旦我们得到了粗略的场景几何和外观估计，我们使用一个自动编码器网络来渲染最终的目标图像。使用粗略特征生成高分辨率的新视图是一个挑战。因此，我们利用[18]的深度平面重采样技术来获得具有与目标视图完全相同的空间分辨率的近深度特征F'。对于从目标相机发射的每个射线，我们在给定射线的预测深度值周围采样J个点。我们将这些点反向投影到每个输入相机中，并通过双线性插值获得它们对应的特征。

F'中每个像素F' ∈ RH×W ×JC/4是多视图变形特征的加权和，权重定义为每个输入视点处xk的逆深度。随着粗略辐射场预测器在预测深度图方面的改进，特征融合方法也是如此。有关更多详细信息，请参考[18]。

**UNet渲染器：** 为了在高分辨率下渲染新视图，我们将粗略特征F上采样以匹配原始目标的空间分辨率，然后与F'连接，再输入到一个完全卷积的UNet神经渲染器中，该渲染器包含三个上采样和下采样卷积层。我们没有使用编码器和解码器之间的跳跃连接，而是采用了几个带有快速傅里叶卷积（Fast Fourier Convolutions）的残差块。这些块具有图像范围的接受场，可以有效地结合多尺度特征F和F'来渲染高分辨率的新图像I+RGB。由于编码器和解码器之间没有跳跃连接，Unet模型也由于参数数量的减少而变得更小、更高效。

## C. 损失函数
我们使用细重建损失Lfine训练粗略辐射场预测器和自动编码器网络，这是I+RGB和地面真实图像之间的L1损失和感知损失的组合。也可以添加一个1×1的卷积层，将粗略特征F转换为低分辨率RGB颜色图像IRGB。输出一个中间的粗略新视图允许我们使用IRGB（见图3右）和下尺度地面真实图像之间的重建L1损失项Lcoarse来训练CG-NeRF。这个损失项还有助于规范粗略辐射场学习，而不依赖于深度监督。为了确保IRGB和I+RGB在视觉上与彼此一致，我们遵循[20]的双判别器设置，增加了一个铰链GAN损失LGAN。与在三个通道图像上进行判别不同，我们使用六通道真实和假图像进行对抗性训练。假图像IRGB在与I+RGB连接之前被上采样。地面真实图像也在与原始图像连接之前被下采样。

我们训练CG-NeRF的总光度损失计算为Ltotal = Lfine + Lcoarse + λLGAN。

# IV. 实验
在本节中，我们评估了CG-NeRF的性能，并将其生成的新视图与最先进的方法进行了比较。
## A. 实现细节
模型细节：模型使用Adam优化器进行训练，判别器的学习率为0.004，粗糙辐射场预测器和神经渲染器的学习率均为0.001，动量参数为(0, 0.9)。 $\lambda = 0.5, C = 64, K = 64, J = 4, N = 3, W = 640, H = 512$ ，MobileViT块有4个注意力头以加快推理速度。我们使用tiny-cuda-nn库的Pytorch扩展来实现全连接层。CG-NeRF的其他模块使用原生Pytorch实现。我们首先使用四个V100 GPU和16的批量大小，从零开始训练模型16小时。然后在单个V100 GPU上进行10-15分钟的微调，为每个测试场景实现最先进的结果。我们还尝试在消费级RTX 2080TI GPU上进行15分钟的微调，观察到在视图合成质量方面没有显著差异。
视图选择：我们遵循[51]中的视图选择方法，为每个目标图像选择最近的10个源图像。我们首先对每个训练场景运行标准结构从运动方法[52]，以估计数据集中每个图像的粗略深度图。然后，我们使用已知的相机内参和外参，将新视图中具有有效深度值的像素投影到每个输入图像中。对于每个新视图，我们选择最接近的10个输入图像，这些图像具有最多的有效投影。由于我们的方法受到GPU内存的限制，我们随后从10个最近视图中随机采样N个视图。在每个训练步骤中，输入到网络的视图数量N进一步在3-5之间均匀采样。
然而，当一组稀疏的N个附近视图只能覆盖场景的一小部分时，执行视图外推是具有挑战性的。我们通过在微调阶段增加N的数量来解决这个问题。我们在其他一般化的视图合成基线方法[10]、[21]上应用相同的视图选择策略，并在以下部分中报告与它们的比较结果。我们还在表V中更详细地讨论了增加N对训练性能的影响。
## B. 实验
*数据集*：我们在DTU[22]数据集上训练CG-NeRF以学习一个可推广的网络。DTU是一个MVS数据集，由在7种不同光照条件下的49个位置扫描的100多个场景组成。从49个相机姿态中，我们选择了10个作为视图合成的目标，并使用其余的姿态选择源图像。我们使用DTU数据集的测试集评估我们预训练模型的性能。为了进一步将CG-NeRF与最先进的方法进行比较，我们在Synthetic-NeRF[7]、Forward-Facing[23]和Tanks & Temples[24]数据集上测试了它，这些数据集的场景和视图分布与我们的训练集不同。每个场景包括12到62张图像，其中这些图像的1/8被留作测试。

*基线*：在这次评估中，我们将CG-NeRF与一般化的视图合成和纯场景优化方法进行了比较。前者方法[8]、[10]、[21]有无每个场景优化来预测新视图。我们使用他们提供的代码在DTU数据集[22]上训练它们，然后对每个测试场景进行微调，以进行公平比较。此外，我们还将我们的方法与最近的特定场景合成方法如Instant-NGP[17]、PointNeRF[13]和DVGO[16]进行了比较。我们使用他们的公共代码训练特定场景模型，并定性和定量地比较了它们与我们方法生成的新视图。我们使用四个V100 GPU来训练和测试所有基线，并与我们的方法进行比较，以进行公平比较。

*指标*：我们报告了CG-NeRF和其他最先进方法的PSNR、SSIM和感知相似性(LPIPS)[53]。我们在表I中总结了CG-NeRF和其他最先进方法的定量和定性结果，并使用来自四个不同数据集[7]、[22]、[23]、[24]的样本在表1、图4和图5中进行了展示。更多定性结果请参见补充视频。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/8e9be34951b24d1a8f4ea504d70b6a53.png" width="70%" /> </div>
<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/c2aabda0e5cb403c8e4ef205f075e315.png" width="70%" /> </div>


*在看到的数据集上的测试*：我们首先在DTU数据集的测试集上评估CG-NeRF。由于我们的方法是在相同数据集的训练集上训练的，我们观察到预训练的CG-NeRF和每个场景优化的CG-NeRF†都能在未见过的测试场景中重建准确的新视图，如图4的第二和第三列所示。此外，它们在定量和定性上都优于其他最先进的方法。IBRNet[21]和MVSNeRF[10]的直接推理网络无法产生窗户的真实纹理和玩具角色鼻尖的反射。这两个基线倾向于预测模糊的结果，并且无法恢复细节，如图4的放大插图所示。由于这些方法预测每个像素的高分辨率新视图来自一个低分辨率的特征体积，它们的MLP网络必须解决两个困难的任务，即图像合成和超分辨率。相反，我们通过两个不同的网络来解决这两个任务：一个估计粗糙的辐射特征，另一个使用对抗性训练和基于卷积的神经渲染器来细化它们。如图4所示，如果两个模型都针对每个测试场景进行了优化，IBRNet*和MVSNeRF†的渲染结果会更清晰。请注意，它们都需要大约24分钟到1小时的时间来实现清晰的结果，但仍然没有我们的在15分钟内为所有测试场景微调的方法准确。

*在未见过的数据集上的测试*：为了进一步测试我们方法在未见数据上的泛化能力，我们在三个合成和真实数据集上进行了实验。如图5的第二列所示，CG-NeRF能够在所有测试新视图上产生合理的结果，这些新视图与训练时使用的DTU图像非常不同。尽管没有见过这些测试图像，CG-NeRF模型仍然展现出与训练了相同时间的MVSNeRF†变体相竞争的结果。在将近一个小时的微调后，IBRNet∗显著提高了其结果，但仍然无法像我们的方法那样估计出同样准确的新视图。在四分之一小时的优化后，CG-NeRF†在这些未见过的合成和真实数据集上产生了比MVSNeRF†和IBRNet∗更清晰、更逼真的新视图。尽管在测试场景上进行了训练，但这两种基线方法都无法渲染出如此精细的细节，因为它们只在随机渲染和地面真实像素之间使用L2颜色损失。在Synthetic-NeRF数据集的船场景中，我们优化的CG-NeRF†模型可以渲染出其他方法生成的新视图中不可见的薄结构。在Forward-Facing数据集的真实花朵和trex场景中，我们的生成图像中可以看到高频细节。此外，我们还可以观察到，在Tank&Temples数据集的第二个示例中，卡车门上的文字更清晰。尽管我们的预训练CG-NeRF模型无法恢复这些细节，但由于我们在训练期间应用的铰链GAN损失 $L_{GAN}$ ，渲染结果可以大大改善。请注意，由于生成的图像分辨率有限，其他基线方法不容易进行对抗性训练。这对我们的方法不是问题，因为我们可以高效地渲染整个新视图，而不必担心其他基线不可避免的内存不足问题。

我们还与最先进的特定场景视图合成PointNeRF[13]模型进行了比较。由于该方法不是为一般化视图合成而设计的，因此性能比CG-NeRF差。然而，我们观察到在测试数据上微调的PointNeRF†模型的性能有了显著提高。如图5的最后一列所示，PointNeRF†可以渲染细节，但生成的新视图仍然不如我们在所有测试场景中的准确。在Synthetic-NeRF数据集的具有挑战性的焦点场景中，我们的方法可以比PointNeRF†更清晰地渲染叶子，后者从点云渲染高分辨率的新视图，其中每个点的特征是从低分辨率的图像特征中插值得到的。尽管使用了内存昂贵的点云表示，PointNeRF†仍然无法渲染新视图的高质量细节，特别是当我们近距离查看生成图像的内容时。

*渲染速度*：在本节中，我们比较了完整的CG-NeRF模型和其他视图合成方法之间的渲染速度。总的来说，我们的方法不仅产生更好的新视图，而且比以前的工作渲染得更快。pixelNeRF[8]和IBRNet[21]需要超过半分钟的时间来渲染单个新图像，因为该方法使用耗时的基于MLP的架构进行多视图聚合，并且它还继承了NeRF的慢渲染。此外，它还需要几个小时的训练时间，但表现仍然不如我们的方法。

基于点的方法PointNeRF[13]通过直接从其混合隐式-显式体素表示中渲染新视图来提高速度。然而，该方法仍然缓慢，无法以交互式速率渲染新视图。如表II所示，我们5-15分钟微调的CG-NeRF模型不仅优于最先进的特定场景快速视图合成方法[16]、[17]，而且至少比它们快3倍。我们发现，使用所提出的全融合MLP和基于卷积的神经渲染器渲染整个新视图比使用NeRF及其变体的深度MLP模型顺序渲染单个像素更快。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/9eb6fba29da64019ba9d04b4fc3536b8.png" width="70%" /> </div>

# V. 消融研究
## A. 架构设计
表IV和图6总结了使用Forward-Facing数据集[24]的测试集对CG-NeRF进行不同架构选择的定量和定性结果。我们首先定义了一个“随机射线”变体的CG-NeRF，在训练期间估计高分辨率新视图的随机采样像素[10]、[21]。独立渲染每个像素会导致预测的新视图出现可见的伪影和模糊。如果我们使用基于卷积的神经渲染器来估计整个新视图，渲染结果会更好。然而，这个模型不能产生合理的目标视图，因为它们仍然包含不正确的几何形状和渲染不佳的镜面反射区域。通过使用粗糙重建损失 $L_{coarse}$ 对我们的模型进行正则化，我们解决了上述问题，并观察到新视图的质量有了极大的提高。最后，我们发现添加一个铰链GAN损失 $L_{GAN}$ 和[20]中的双判别器有助于我们实现最先进的结果，如Fig. 6的最后一列所示。我们在补充视频中提供了更多的比较结果。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/3d8f7ce8df5146878c2235724acef70c.png" width="70%" /> </div>

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/3417046f28d940a494a97fe627cb0407.png" width="70%" /> </div>

## B. 渲染大型4K新视图
由于我们使用基于卷积的神经渲染器来获取高分辨率图像，CG-NeRF可以接受Forward-Facing数据集[23]的非常大的4K输入图像，并生成新视图。在表III中，我们在800x800和4K分辨率下将我们的方法与其他NeRF变体进行了比较。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/aa7524123c4a49d1845308c077e8b094.png" width="70%" /> </div>


我们首先进行一个实验，测试我们的神经渲染器是否能够改进现有的视图合成方法，如PointNeRF[13]和Instant-NGP[17]，在测试标准800x800分辨率下的表现。带有‡符号的方法表明，它们被训练以产生更小的新视图，然后后来使用我们的神经渲染器上采样到原始大小。给定相同的训练时间，这两种基线方法产生了几乎相似的新视图，但渲染时间显著提高。这进一步突出了我们的新型神经渲染器对最近提出的方法的有用性，它可以轻松地插入现有系统中。

我们还发现，直接优化PointNeRF[13]和Instant-NGP[17]在4K图像上需要超过20GB的GPU内存和更长的训练时间才能获得良好的合成结果。因此，我们应用了上述的上采样策略来减少内存占用。相比之下，我们的方法只需要大约5GB的GPU内存来合成高质量的4K新视图。我们不是从头开始训练CG-NeRF‡模型，而是微调我们的通用CG-NeRF模型并输出4K图像。实验结果在表III中显示，优化基于卷积的神经渲染器可以在测试标准800x800和非常高的4K分辨率下提高新视图的合成质量。此外，渲染单个4K新图像需要2.5秒，但我们的方法仍然比其他基线快得多。
# C. 时空一致性
从补充材料中可以看出，我们的通用和微调后的CG-NeRF模型使用学习到的编码器-解码器结构渲染具有多视图一致性的高保真度新视图。在最近的3D生成NeRF基础方法[20]、[54]中也观察到了类似的结果，我们可以从2D低分辨率特征映射中产生高分辨率3D一致性新视图。在这项工作中，我们遵循最近提出的EG3D[20]的设计，该设计使用双判别器来强制执行高分辨率和低分辨率输出之间的一致性结果。[20]的上采样神经渲染器与我们提出的编码器-解码器网络类似，但我们使用一组稀疏的输入视图来条件化合成过程。

如Fig. 2所示，我们还使用聚合变形特征作为神经渲染器的输入。这些变形特征与粗糙辐射场特征一致，因为我们利用预测的粗糙深度图执行特征融合，如第III-B节所述。因此，我们添加了一个正则化损失 $L_{coarse}$ 来训练粗糙辐射场预测器。我们发现，使用一个简单而有效的损失函数不仅可以提高低分辨率和高分辨率新视图的质量，还可以提高连续新视图之间的时间一致性。没有 $L_{coarse}$ ，预测的新视图在边界附近包含显著的伪影，并且由于使用2D Unet渲染器独立渲染每个新视点，因此时间一致性也较差（见图7）。通过强制网络估计准确的下采样新视图，我们的方法可以学习在更高分辨率下产生一致的特征。我们还尝试了[54]的改进设计在我们的流水线上，并观察到类似的结果。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/44dc1c0c919a4c05a5465dbb7971d364.png" width="70%" /> </div>

## D. 输入视图的数量

在表V中，我们评估了我们方法的性能，使用Tanks和Temples[24]数据集，随着源图像数量的增加。我们报告了使用多达10个源图像的SSIM和LPIPS指标。我们观察到CG-NeRF在有7个输入视图时表现最佳，然后结果变差。当参考和目标姿态彼此相距较远时，不准确的回归深度图将导致新视图的准确性降低。因此，拥有接近目标视图且自遮挡较少的视图对于合成新视图至关重要。如果很难收集到目标视图周围的视图，那么增加与目标视图有重叠视角的视图也是必要的。

<div align=center>   <img src="https://img-blog.csdnimg.cn/direct/b1facb740dc64698b4d032c031f48c98.png" width="70%" /> </div>

# VI. 结论
我们提出了CG-NeRF，这是一种新方法，用于解决从稀疏和非结构化的输入图像集合进行新视角合成的挑战性问题。由于其粗糙的神经辐射场预测器和基于卷积的神经渲染器，CG-NeRF能够不依赖任何额外的显式数据结构来生成目标视图的所有像素。此外，它还实现了高效的每个场景的优化，仅需要10-15分钟，从而实现与最近需要几个小时训练的最新技术方法相媲美甚至超越的渲染质量。

# 声明
本文内容为论文学习收获分享，受限于知识能力，本文队员问的理解可能存在偏差，最终内容以原论文为准。本文信息旨在传播和学术交流，其内容由作者负责，不代表本号观点。文中作品文字、图片等如涉及内容、版权和其他问题，请及时与我们联系，我们将在第一时间回复并处理。
