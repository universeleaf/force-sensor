function scenes=contact_demo_scenes()
% Reproducible single-contact cases. All loads/planes below belong ONLY to
% truth generation and scoring, never the estimator packet or its config.
a=struct('id','ceiling_hook','title','弯钩杆向上顶面','lengthMm',200, ...
    'straightMm',120,'radiusMm',30,'stiffnessScale',1, ...
    'pushMm',[10 16 20],'tipForceXZ',[1;-1], ...
    'planePointXZ',[0;160],'planeNormalXZ',[0;-1], ...
    'worldRotationYDeg',0,'frictionMu',0,'sliding',false, ...
    'slipMm',0.02,'curvatureNoiseStd',0,'planeNoiseStdMm',0, ...
    'seed',93,'description','视频近似弯钩几何，独立末端载荷，向上推进。');
scenes=repmat(a,1,6);
scenes(2).id='side_wall'; scenes(2).title='侧向墙面接触';
scenes(2).worldRotationYDeg=90; scenes(2).pushMm=[12 17 21];
scenes(2).tipForceXZ=[0.4;-0.6];
scenes(2).description='整体旋转 90 度后侧推，改变末端载荷；仍是平面内力学。';
scenes(3).id='inclined_plane'; scenes(3).title='倾斜环境接触';
scenes(3).planeNormalXZ=[sind(12);-cosd(12)];
scenes(3).planePointXZ=[0;145]; scenes(3).pushMm=[12 17 21];
scenes(3).tipForceXZ=[0.5;-0.6];
scenes(3).description='杆基座方向不变，环境法向相对杆倾斜 12 度。';
scenes(4).id='long_soft_rod'; scenes(4).title='较长较软弯杆';
scenes(4).lengthMm=240; scenes(4).straightMm=144; scenes(4).radiusMm=36;
scenes(4).stiffnessScale=0.65; scenes(4).planePointXZ=[0;192];
scenes(4).pushMm=[12 20 26]; scenes(4).tipForceXZ=[0.4;-0.4];
scenes(4).worldRotationYDeg=180;
scenes(4).description='240 mm 杆，弯曲刚度为参考的 65%，翻转后向下压水平面。';
for j=5:6
    scenes(j).sliding=true; scenes(j).frictionMu=0.3;
    scenes(j).pushMm=[18 20 22];
end
scenes(5).id='sliding_clean'; scenes(5).title='沿顶面滑动';
scenes(5).description='独立连续平衡上的指定滑动分支；每对历史样本滑移 0.02 mm。';
scenes(6).id='sliding_noisy'; scenes(6).title='相同滑动＋测量噪声';
scenes(6).curvatureNoiseStd=5e-5; scenes(6).planeNoiseStdMm=0.1;
scenes(6).description='与无噪声滑动相同真值，加入弯曲噪声及 0.1 mm 平面偏移噪声。';
end
