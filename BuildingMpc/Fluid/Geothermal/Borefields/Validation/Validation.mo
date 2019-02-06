within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model Validation
  extends Modelica.Icons.Example;
  package Medium = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=283.15, X_a=0.30);
  INFRAX.Data.Parameters.BorefieldData.INFRAX_bF iNFRAX_bF
    annotation (Placement(transformation(extent={{-84,-20},{-64,0}})));
  TwoUTube borFieNLC(
    borFieDat=iNFRAX_bF,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    show_T=true,
    TExt0_start=13.3 + 273.15,
    z0=10,
    dT_dz=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dynFil=true,
    nSeg=10,
    lay(nSta=10))
    annotation (Placement(transformation(extent={{-34,0},{6,40}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TInNLC(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-64,10},{-44,30}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutNLC(
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0,
    T(start=293.15 + 13.3))
           annotation (Placement(transformation(extent={{16,10},{36,30}})));
  IDEAS.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    use_T_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-94,10},{-74,30}})));
  IDEAS.Fluid.Sources.Boundary_pT sink(redeclare package Medium = Medium,
      nPorts=1)
    annotation (Placement(transformation(extent={{96,10},{76,30}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlowNLC(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{46,10},{66,30}})));
public
  IDEAS.Fluid.Sources.Boundary_pT sou1(
    redeclare package Medium = Medium,
    use_T_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-94,-50},{-74,-30}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TInEmu(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-64,-50},{-44,-30}})));
  IBPSA.Fluid.Geothermal.Borefields.TwoUTubes borFieEmu(
    borFieDat=iNFRAX_bF,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    show_T=true,
    r=cat(
        1,
        borFieNLC.lay[1].rC,
        {borFieNLC.r_b}),
    TExt0_start=13.3 + 273.15,
    nbTem=borFieNLC.lay[1].nSta + 1,
    dT_dz=0)
    annotation (Placement(transformation(extent={{-34,-60},{6,-20}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutEmu(
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0,
    T(start=273.15 + 13.3))
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
      nPorts=1)
    annotation (Placement(transformation(extent={{96,-50},{76,-30}})));
public
  Modelica.Blocks.Math.Add err1(k1=-1)
    annotation (Placement(transformation(extent={{66,-20},{86,0}})));
  Modelica.Blocks.Interfaces.RealOutput error_nl
    annotation (Placement(transformation(extent={{100,-20},{120,0}})));
  Modelica.Blocks.Sources.Constant TSup(k=273.15 + 5)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  IDEAS.Utilities.Time.ModelTime modTim
    annotation (Placement(transformation(extent={{-30,60},{-10,80}})));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable2D(
    tableName="data",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://INFRAX/Resources/Validation/borefield_validation.txt"),
    tableOnFile=false,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    table=[0,1.25; 900,3.75; 1800,0; 2700,5; 3600,1.25; 7200,3.75; 10800,0;
        14400,5; 28800,1.25; 43200,3.75; 86400,0; 172800,5; 259200,1.25; 604800,
        3.75])
    annotation (Placement(transformation(extent={{10,60},{30,80}})));
equation
  connect(TInNLC.port_b,borFieNLC. port_a)
    annotation (Line(points={{-44,20},{-34,20}},   color={0,127,255}));
  connect(borFieNLC.port_b,TOutNLC. port_a)
    annotation (Line(points={{6,20},{16,20}},    color={0,127,255}));
  connect(TOutNLC.port_b,mFlowNLC. port_a)
    annotation (Line(points={{36,20},{46,20}},   color={0,127,255}));
  connect(mFlowNLC.port_b,sink. ports[1])
    annotation (Line(points={{66,20},{76,20}},   color={0,127,255}));
  connect(sou.ports[1],TInNLC. port_a)
    annotation (Line(points={{-74,20},{-64,20}},   color={0,127,255}));
  connect(sou1.ports[1],TInEmu. port_a)
    annotation (Line(points={{-74,-40},{-64,-40}}, color={0,127,255}));
  connect(TInEmu.port_b,borFieEmu. port_a)
    annotation (Line(points={{-44,-40},{-34,-40}}, color={0,127,255}));
  connect(borFieEmu.port_b,TOutEmu. port_a)
    annotation (Line(points={{6,-40},{16,-40}},  color={0,127,255}));
  connect(TOutEmu.port_b,mFlowEmu. port_a)
    annotation (Line(points={{36,-40},{46,-40}}, color={0,127,255}));
  connect(mFlowEmu.port_b,sink1. ports[1])
    annotation (Line(points={{66,-40},{76,-40}}, color={0,127,255}));
  connect(err1.y, error_nl)
    annotation (Line(points={{87,-10},{110,-10}}, color={0,0,127}));
  connect(TOutNLC.T, err1.u1) annotation (Line(points={{26,31},{38,31},{38,-4},{
          64,-4}}, color={0,0,127}));
  connect(TOutEmu.T, err1.u2)
    annotation (Line(points={{26,-29},{26,-16},{64,-16}}, color={0,0,127}));
  connect(TSup.y, sou.T_in) annotation (Line(points={{-59,70},{-42,70},{-42,46},
          {-96,46},{-96,24}}, color={0,0,127}));
  connect(TSup.y, sou1.T_in) annotation (Line(points={{-59,70},{-42,70},{-42,46},
          {-96,46},{-96,-36}}, color={0,0,127}));
  connect(modTim.y, combiTable2D.u)
    annotation (Line(points={{-9,70},{8,70}}, color={0,0,127}));
  connect(combiTable2D.y[1], mFlowNLC.m_flow_in)
    annotation (Line(points={{31,70},{50,70},{50,28}}, color={0,0,127}));
  connect(combiTable2D.y[1], mFlowEmu.m_flow_in) annotation (Line(points={{31,
          70},{42,70},{42,-28},{50,-28},{50,-32}}, color={0,0,127}));
  annotation (experiment(
      StopTime=604800,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"));
end Validation;
