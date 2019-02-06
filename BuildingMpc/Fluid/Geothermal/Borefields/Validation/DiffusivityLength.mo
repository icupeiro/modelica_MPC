within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model DiffusivityLength
  extends Modelica.Icons.Example;
  package Medium = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=283.15, X_a=0.30);
  IDEAS.Fluid.Sensors.TemperatureTwoPort TInEmu(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-64,-50},{-44,-30}})));
  IBPSA.Fluid.Geothermal.Borefields.OneUTube  borFieEmu(
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    show_T=true,
    dT_dz=0,
    borFieDat=borFieDat,
    TExt0_start=10 + 273.15,
    z0=0,
    r={1,2,3,6},
    nbTem=4)
    annotation (Placement(transformation(extent={{-34,-60},{6,-20}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutEmu(
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    tau=0,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    T(start=273.15 + 10))
           annotation (Placement(transformation(extent={{16,-50},{36,-30}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlowEmu(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{46,-50},{66,-30}})));
public
  IDEAS.Fluid.Sources.Boundary_pT sink1(redeclare package Medium = Medium,
      nPorts=2,
    p=200000)
    annotation (Placement(transformation(extent={{96,-50},{76,-30}})));
  parameter IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(soiDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(kSoi=3.5, cSoi=
        790), conDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example(
        hBor=80,
        nBor=1,
        cooBor={{0,0}}))
    annotation (Placement(transformation(extent={{-48,0},{-28,20}})));
  Modelica.Blocks.Sources.Constant TSup1(k=0.3)
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume vol(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    nPorts=2,
    redeclare package Medium = Medium,
    T_start=273.15 + 10,
    V=1)    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-70,12})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow(Q_flow=1000,
      T_ref=283.15)
    annotation (Placement(transformation(extent={{-28,38},{-48,60}})));
  Modelica.Blocks.Sources.RealExpression[4] diffusionTime(y={borFieEmu.r[i]*
        borFieEmu.r[i]/4/borFieDat.soiDat.aSoi/3600 for i in 1:4})
    annotation (Placement(transformation(extent={{40,-90},{60,-70}})));
  Modelica.Blocks.Sources.RealExpression ts(y=time/(borFieDat.conDat.hBor*
        borFieDat.conDat.hBor/9/borFieDat.soiDat.aSoi))
    annotation (Placement(transformation(extent={{-40,-106},{-20,-86}})));
equation
  connect(TInEmu.port_b,borFieEmu. port_a)
    annotation (Line(points={{-44,-40},{-34,-40}}, color={0,127,255}));
  connect(borFieEmu.port_b,TOutEmu. port_a)
    annotation (Line(points={{6,-40},{16,-40}},  color={0,127,255}));
  connect(TOutEmu.port_b,mFlowEmu. port_a)
    annotation (Line(points={{36,-40},{46,-40}}, color={0,127,255}));
  connect(mFlowEmu.port_b,sink1. ports[1])
    annotation (Line(points={{66,-40},{72,-40},{72,-38},{76,-38}},
                                                 color={0,127,255}));
  connect(TSup1.y, mFlowEmu.m_flow_in)
    annotation (Line(points={{41,50},{50,50},{50,-32}}, color={0,0,127}));
  connect(sink1.ports[2], vol.ports[1]) annotation (Line(points={{76,-42},{76,80},
          {-90,80},{-90,14},{-80,14}}, color={0,127,255}));
  connect(vol.ports[2], TInEmu.port_a) annotation (Line(points={{-80,10},{-80,-40},
          {-64,-40}}, color={0,127,255}));
  connect(vol.heatPort, fixedHeatFlow.port)
    annotation (Line(points={{-70,22},{-70,49},{-48,49}}, color={191,0,0}));
  annotation (experiment(
      StopTime=604800,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"));
end DiffusivityLength;
