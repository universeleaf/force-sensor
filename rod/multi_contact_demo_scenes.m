function scenes = multi_contact_demo_scenes()
% Fixed environments; forces and contact locations are solver outputs.
a=struct('id','s_channel_two_contact','title','S 形杆双侧通道', ...
    'description','S 形杆同时压住左右两壁，基座横移使两个接触反力重新分配。', ...
    'lengthMm',140,'segmentLengthMm',70,'intrinsicCurvature',0.02, ...
    'baseAngleRad',-0.7,'planePointXZ',[-10 10;35 105], ...
    'planeNormalXZ',[1 -1;0 0],'contactPlaneIndex',[1 2], ...
    'baseMotionMm',linspace(-0.6,0.6,12),'baseOriginXZ',[0;0], ...
    'tipForceXZ',[0.10;-0.08],'curvatureNoiseStd',0,'seed',601);
scenes=repmat(a,1,4);
scenes(2).id='tapered_channel_two_contact';
scenes(2).title='收窄通道双接触';
scenes(2).description='两侧壁法向不同，通道向前收窄；同时求解两个接触点和反力。';
scenes(2).planeNormalXZ=[1 -1;-0.05 -0.05];
scenes(2).seed=602;
scenes(3).id='serpentine_three_contact';
scenes(3).title='蛇形杆三接触';
scenes(3).description='蛇形杆依次接触左壁、右壁和左壁；每个接触独立报告力和位置。';
scenes(3).lengthMm=210;
scenes(3).contactPlaneIndex=[1 2 1];
scenes(3).tipForceXZ=[-0.08;-0.05];
scenes(3).seed=603;
scenes(4)=scenes(3);
scenes(4).id='serpentine_three_contact_noisy';
scenes(4).title='蛇形杆三接触（曲率噪声）';
scenes(4).description='相同三接触力学真值，24 个曲率观测叠加标准差 2.5e-5 /mm 的高斯噪声。';
scenes(4).curvatureNoiseStd=2.5e-5;
scenes(4).seed=604;
end
