within BuildingMpc.Examples.Optimizations;
model Case900GEOTABS_cooling_noBuffTank
  "Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
  extends Modelica.Icons.Example;
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  Modelica.Blocks.Sources.RealExpression optVar1(y=
        mpcCase900GEOTABS_cooling_noBuffTank.yOpt[1])
    annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    dp_nominal=0,
    m_flow_nominal=0.5,
    A_floor=boundaryWall.A)
    annotation (Placement(transformation(extent={{-6,-70},{14,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    p=200000,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
  Modelica.Blocks.Sources.RealExpression optVar3(y=
        mpcCase900GEOTABS_cooling_noBuffTank.yOpt[3])
    annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
protected
  IDEAS.Fluid.Movers.FlowControlled_m_flow   m_flow_tabs(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final m_flow_small=1e-04,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    nominalValuesDefineDefaultPressureCurve=true) "Pressure source"
    annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
protected
  IDEAS.Fluid.Movers.FlowControlled_m_flow   m_flow_source(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final m_flow_small=1e-4,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    nominalValuesDefineDefaultPressureCurve=true,
    m_flow_nominal=0.5)      "Pressure source"
    annotation (Placement(transformation(extent={{40,62},{60,82}})));
protected
  IDEAS.Fluid.Movers.FlowControlled_m_flow m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final m_flow_small=1e-04,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    nominalValuesDefineDefaultPressureCurve=true,
    inputType=IDEAS.Fluid.Types.InputType.Stages) "Pressure source"
    annotation (Placement(transformation(extent={{128,-136},{108,-116}})));
public
  Modelica.Blocks.Sources.RealExpression optVar2(y=
        mpcCase900GEOTABS_cooling_noBuffTank.yOpt[2])
    annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,-50})));
  IBPSA.Fluid.HeatPumps.ScrollWaterToWater
                                       heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    dp2_nominal=0,
    dp1_nominal=0,
    m1_flow_nominal=0.5,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    enable_variable_speed=false,
    scaling_factor=3000/10200,
    datHeaPum=
        IBPSA.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),
    m2_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{60,-44},{80,-64}})));
  MPCs.MpcCase900GEOTABS_cooling_noBuffTank
                         mpcCase900GEOTABS_cooling_noBuffTank(
                                                   stateEstimationType=
        UnitTests.MPC.BaseClasses.StateEstimationType.Perfect)
    annotation (Placement(transformation(extent={{-94,20},{-74,40}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{158,-134},{140,-118}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = IDEAS.Media.Water,
    m_flow_nominal=0.5,
    nPorts=4,
    T_start=308.15,
    V=1)
    annotation (Placement(transformation(extent={{52,-92},{72,-72}})));
  Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=-3, uHigh=1)
    annotation (Placement(transformation(extent={{-16,4},{4,24}})));
  Modelica.Blocks.Math.Add add(k2=-1)
    annotation (Placement(transformation(extent={{-46,4},{-26,24}})));
  Modelica.Blocks.Math.BooleanToInteger booleanToInteger
    annotation (Placement(transformation(extent={{90,-112},{110,-92}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{16,-94},{-4,-74}})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    m2_flow_nominal=0.5,
    dp1_nominal=0,
    dp2_nominal=0,
    eps=0.9,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    m1_flow_nominal=0.5)      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={74,-10})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveMotor(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    l=0,
    m_flow_nominal=0.5,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (Placement(transformation(extent={{-20,-112},{-40,-92}})));
  Modelica.Blocks.Sources.RealExpression optVar4(y=if
        mpcCase900GEOTABS_cooling_noBuffTank.yOpt[4] < 0.75 then 0 else 1)
    annotation (Placement(transformation(extent={{-96,-94},{-76,-74}})));
  IDEAS.Fluid.FixedResistances.Junction jun(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    each m_flow_nominal=0.5*ones(3),
    each dp_nominal=0*ones(3))
    annotation (Placement(transformation(extent={{24,-68},{40,-52}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(
    filDat=IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(),
    soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(),
    conDat=IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example(
        mBor_flow_nominal=0.5,
        dp_nominal=0,
        nBor=1,
        cooBor={{0,0}}))
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveMotor1(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    l=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=1.2)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={54,-30})));
  IDEAS.Fluid.FixedResistances.Junction jun1(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    each dp_nominal=0*ones(3),
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Entering,
    each m_flow_nominal=1.2*ones(3))
    annotation (Placement(transformation(extent={{-8,-8},{8,8}},
        rotation=90,
        origin={54,4})));

    Real simState[31];
    Real borCapFil[20];
    Real borCapWat[20];

  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare package Medium = Media.DryAir,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir airModel(
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState),
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
      conTypFlo,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypA,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypB,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypC,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypD,
    hasWinA=true,
    fracA=0,
    redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
      shaTypA,
    hasWinB=false,
    hasWinC=false,
    hasWinD=false,
    bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    aziA=IDEAS.Types.Azimuth.S,
    mSenFac=0.822,
    l=8,
    w=6,
    A_winA=12,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingA,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
    redeclare IDEAS.Buildings.Components.InterzonalAirFlow.AirTight
      interzonalAirFlow,
    T_start=295.15)
    annotation (Placement(transformation(extent={{-8,-20},{12,0}})));

  IDEAS.Buildings.Components.BoundaryWall boundaryWall(
    inc=IDEAS.Types.Tilt.Floor,
    azi=rectangularZoneTemplate.aziA,
    A=rectangularZoneTemplate.A,
    redeclare Data.HeavyFloorTABS constructionType)
                                 annotation (Placement(transformation(
        extent={{-6,-10},{6,10}},
        rotation=90,
        origin={4,-38})));
  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        Media.DryAir)
    annotation (Placement(transformation(extent={{-34,30},{-14,50}})));
  IBPSA.Fluid.Geothermal.Borefields.OneUTube       multipleBorehole(
    redeclare package Medium = IDEAS.Media.Water,
    borFieDat=borFieDat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dynFil=true,
    TExt0_start=(273.15 + 10.8),
    show_T=true,
    nbTem=11,
    r=cat(
        1,
        lay.rC,
        {lay.r_b}))
    annotation (Placement(transformation(extent={{52,28},{32,48}})));
  IBPSA.Fluid.Geothermal.Borefields.BaseClasses.HeatTransfer.Cylindrical lay(
    soiDat=borFieDat.soiDat,
    h=borFieDat.conDat.hBor/10,
    r_a=borFieDat.conDat.rBor,
    r_b=2*sqrt(borFieDat.soiDat.aSoi*604800),
    TInt_start=283.95,
    TExt_start=283.95)
    annotation (Placement(transformation(extent={{172,46},{192,66}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T(
        displayUnit="K") = 300)
    annotation (Placement(transformation(extent={{114,64},{134,84}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=3000)
    annotation (Placement(transformation(extent={{182,82},{202,102}})));
  Modelica.Blocks.Continuous.Integrator integrator
    annotation (Placement(transformation(extent={{138,24},{158,44}})));
  Modelica.Blocks.Interfaces.RealOutput energy_use
    annotation (Placement(transformation(extent={{200,22},{220,42}})));
  Modelica.Blocks.Interfaces.RealOutput slack
    annotation (Placement(transformation(extent={{200,8},{220,28}})));
  Modelica.Blocks.Continuous.Integrator integrator1
    annotation (Placement(transformation(extent={{138,-16},{158,4}})));
  Modelica.Blocks.Sources.RealExpression slack_violation(y=if
        rectangularZoneTemplate.TAir < 295 then 295 - rectangularZoneTemplate.TAir
         elseif rectangularZoneTemplate.TAir > 298 then rectangularZoneTemplate.TAir
         - 298 else 0)
    annotation (Placement(transformation(extent={{110,-28},{130,-8}})));
equation

  simState= {rectangularZoneTemplate.winA.heaCapGla.T,
rectangularZoneTemplate.outA.port_emb[1].T,
rectangularZoneTemplate.outA.extCon.port_a.T,
rectangularZoneTemplate.outA.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[1].surfRad.T,
rectangularZoneTemplate.outA.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outB.port_emb[1].T,
rectangularZoneTemplate.outB.extCon.port_a.T,
rectangularZoneTemplate.outB.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[2].surfRad.T,
rectangularZoneTemplate.outB.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outC.port_emb[1].T,
rectangularZoneTemplate.outC.extCon.port_a.T,
rectangularZoneTemplate.outC.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[3].surfRad.T,
rectangularZoneTemplate.outC.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outD.port_emb[1].T,
rectangularZoneTemplate.outD.extCon.port_a.T,
rectangularZoneTemplate.outD.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[4].surfRad.T,
rectangularZoneTemplate.outD.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outCei.port_emb[1].T,
rectangularZoneTemplate.outCei.extCon.port_a.T,
rectangularZoneTemplate.outCei.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[6].surfRad.T,
rectangularZoneTemplate.airModel.vol.dynBal.U,
rectangularZoneTemplate.radDistr.TRad,
boundaryWall.layMul.port_gain[1].T,
boundaryWall.layMul.port_b.T,
boundaryWall.port_emb[1].T,
boundaryWall.propsBus_a.surfRad.T};

borCapFil = {
multipleBorehole.borHol.intHex[1].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[1].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[2].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[2].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[3].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[3].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[4].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[4].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[5].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[5].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[6].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[6].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[7].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[7].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[8].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[8].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[9].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[9].intResUTub.capFil2.T,
multipleBorehole.borHol.intHex[10].intResUTub.capFil1.T,
multipleBorehole.borHol.intHex[10].intResUTub.capFil2.T};

borCapWat = {
multipleBorehole.borHol.intHex[1].vol1.T,
multipleBorehole.borHol.intHex[1].vol2.T,
multipleBorehole.borHol.intHex[2].vol1.T,
multipleBorehole.borHol.intHex[2].vol2.T,
multipleBorehole.borHol.intHex[3].vol1.T,
multipleBorehole.borHol.intHex[3].vol2.T,
multipleBorehole.borHol.intHex[4].vol1.T,
multipleBorehole.borHol.intHex[4].vol2.T,
multipleBorehole.borHol.intHex[5].vol1.T,
multipleBorehole.borHol.intHex[5].vol2.T,
multipleBorehole.borHol.intHex[6].vol1.T,
multipleBorehole.borHol.intHex[6].vol2.T,
multipleBorehole.borHol.intHex[7].vol1.T,
multipleBorehole.borHol.intHex[7].vol2.T,
multipleBorehole.borHol.intHex[8].vol1.T,
multipleBorehole.borHol.intHex[8].vol2.T,
multipleBorehole.borHol.intHex[9].vol1.T,
multipleBorehole.borHol.intHex[9].vol2.T,
multipleBorehole.borHol.intHex[10].vol1.T,
multipleBorehole.borHol.intHex[10].vol2.T};
for i in 1:31 loop
    mpcCase900GEOTABS_cooling_noBuffTank.uSta[i] = simState[i];
end for;

 for i in 1:20 loop
   mpcCase900GEOTABS_cooling_noBuffTank.uSta[i+31] = borCapFil[i];
 end for;


 for i in 1:10 loop
   for j in 1:10 loop
     mpcCase900GEOTABS_cooling_noBuffTank.uSta[51+i+10*(j-1)] = multipleBorehole.TSoi[j,i];
   end for;
 end for;

 for i in 1:10 loop
   mpcCase900GEOTABS_cooling_noBuffTank.uSta[151+i] = multipleBorehole.TSoi[11,i];
 end for;

 for i in 1:20 loop
   mpcCase900GEOTABS_cooling_noBuffTank.uSta[161+i] = borCapWat[i];
 end for;

  connect(m_flow_tabs.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-24,-60},{-6,-60}}, color={0,127,255}));
  connect(optVar1.y,m_flow_tabs. m_flow_in) annotation (Line(points={{-65,-10},
          {-40,-10},{-40,-48},{-34,-48}}, color={0,0,127}));
  connect(optVar2.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
          {-22,90},{50,90},{50,84}},        color={0,0,127}));
  connect(sink.ports[1],m_flow_tabs. port_a) annotation (Line(points={{-78,-60},
          {-78,-60},{-44,-60}}, color={0,127,255}));
  connect(m_flow_source.port_b, heaPum.port_a2) annotation (Line(points={{60,72},
          {60,72},{88,72},{88,-48},{80,-48}}, color={0,127,255}));
  connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
          {-26,72},{-26,72},{40,72}}, color={0,127,255}));

  connect(optVar3.y, add.u1) annotation (Line(points={{-65,-24},{-58,-24},{-58,
          20},{-48,20}}, color={0,0,127}));
  connect(add.y, hysteresis.u)
    annotation (Line(points={{-25,14},{-18,14}}, color={0,0,127}));
  connect(booleanToInteger.y, heaPum.stage) annotation (Line(points={{111,-102},
          {111,-43},{58,-43},{58,-57}},
                                      color={255,127,0}));
  connect(booleanToInteger.u, hysteresis.y)
    annotation (Line(points={{88,-102},{42,-102},{42,-12},{24,-12},{24,14},{5,
          14}},                                        color={255,0,255}));
  connect(booleanToInteger.y, m_flow_sink.stage) annotation (Line(points={{111,
          -102},{118,-102},{118,-114}},               color={255,127,0}));
  connect(m_flow_sink.port_b, vol.ports[1]) annotation (Line(points={{108,-126},
          {52,-126},{52,-92},{59,-92}}, color={0,127,255}));
  connect(heaPum.port_a1, vol.ports[2]) annotation (Line(points={{60,-60},{52,
          -60},{52,-92},{61,-92}}, color={0,127,255}));
  connect(m_flow_sink.port_a, T_con_out.port_b)
    annotation (Line(points={{128,-126},{140,-126}},
                                                   color={0,127,255}));
  connect(T_con_out.port_a, heaPum.port_b1) annotation (Line(points={{158,-126},
          {158,-60},{80,-60}},            color={0,127,255}));
  connect(vol.heatPort, temperatureSensor.port) annotation (Line(points={{52,
          -82},{35,-82},{35,-84},{16,-84}}, color={191,0,0}));
  connect(temperatureSensor.T, add.u2) annotation (Line(points={{-4,-84},{-14,
          -84},{-14,-32},{-30,-32},{-30,-4},{-48,-4},{-48,8}}, color={0,0,127}));
  connect(threeWayValveMotor.port_a1, vol.ports[3]) annotation (Line(points={{
          -20,-102},{63,-102},{63,-92}}, color={0,127,255}));
  connect(hex.port_b2, threeWayValveMotor.port_a2) annotation (Line(points={{80,-20},
          {80,-40},{170,-40},{170,-144},{-30,-144},{-30,-112}},      color={0,
          127,255}));
  connect(threeWayValveMotor.port_b, m_flow_tabs.port_a) annotation (Line(
        points={{-40,-102},{-44,-102},{-44,-60}}, color={0,127,255}));
  connect(optVar4.y, threeWayValveMotor.ctrl) annotation (Line(points={{-75,-84},
          {-30,-84},{-30,-91.2}}, color={0,0,127}));
  connect(embeddedPipe.port_b, jun.port_1)
    annotation (Line(points={{14,-60},{24,-60}}, color={0,127,255}));
  connect(jun.port_2, vol.ports[4]) annotation (Line(points={{40,-60},{46,-60},
          {46,-92},{65,-92}}, color={0,127,255}));
  connect(jun.port_3, hex.port_a2) annotation (Line(points={{32,-68},{32,-154},{
          190,-154},{190,0},{80,0}},      color={0,127,255}));
  connect(heaPum.port_b2, threeWayValveMotor1.port_b)
    annotation (Line(points={{60,-48},{54,-48},{54,-40}}, color={0,127,255}));
  connect(threeWayValveMotor1.port_a2, hex.port_a1)
    annotation (Line(points={{64,-30},{68,-30},{68,-20}}, color={0,127,255}));
  connect(threeWayValveMotor1.port_a1, jun1.port_1)
    annotation (Line(points={{54,-20},{54,-4}}, color={0,127,255}));
  connect(jun1.port_3, hex.port_b1)
    annotation (Line(points={{62,4},{66,4},{66,0},{68,0}}, color={0,127,255}));
  connect(optVar4.y, threeWayValveMotor1.ctrl) annotation (Line(points={{-75,-84},
          {-30,-84},{-30,-30},{43.2,-30}}, color={0,0,127}));
  connect(embeddedPipe.heatPortEmb[1], boundaryWall.port_emb[1]) annotation (
      Line(points={{4,-50},{24,-50},{24,-38},{14,-38}}, color={191,0,0}));
  connect(boundaryWall.propsBus_a, rectangularZoneTemplate.proBusFlo[1])
    annotation (Line(
      points={{2,-33},{2,-16}},
      color={255,204,51},
      thickness=0.5));
  connect(rectangularZoneTemplate.port_a, bou.ports[1])
    annotation (Line(points={{4,0},{6,0},{6,40},{-14,40}}, color={0,127,255}));
  connect(jun1.port_2, multipleBorehole.port_a) annotation (Line(points={{54,12},
          {56,12},{56,38},{52,38}}, color={0,127,255}));
  connect(multipleBorehole.port_b, m_flow_source.port_a) annotation (Line(
        points={{32,38},{18,38},{18,72},{40,72}}, color={0,127,255}));
  connect(fixedTemperature.port, lay.port_a) annotation (Line(points={{134,74},{
          158,74},{158,56},{172,56}}, color={191,0,0}));
          connect(heatCapacitor.port, lay.port_b);
  connect(energy_use, integrator.y) annotation (Line(points={{210,32},{184,32},
          {184,34},{159,34}}, color={0,0,127}));
  connect(heaPum.P, integrator.u) annotation (Line(points={{81,-54},{108,-54},{
          108,34},{136,34}}, color={0,0,127}));
  connect(integrator1.y, slack) annotation (Line(points={{159,-6},{182,-6},{182,
          18},{210,18}}, color={0,0,127}));
  connect(slack_violation.y, integrator1.u) annotation (Line(points={{131,-18},
          {131,-13},{136,-13},{136,-6}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -180},{200,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},{200,
            100}})),
    experiment(
      StopTime=31536000,
      Interval=900,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=20,
      __Dymola_Algorithm="Euler"),
    Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>", revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),
    __Dymola_Commands(file=
          "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
        "Simulate and plot"),
    __Dymola_experimentSetupOutput(events=false),
    __Dymola_experimentFlags(
      Advanced(
        EvaluateAlsoTop=true,
        GenerateVariableDependencies=false,
        OutputModelicaCode=false,
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001),
      Evaluate=true,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end Case900GEOTABS_cooling_noBuffTank;
