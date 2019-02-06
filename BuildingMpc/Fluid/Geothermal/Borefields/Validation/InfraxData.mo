within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model InfraxData
  extends Modelica.Icons.Example;
  package Medium = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=283.15, X_a=0.30);
  parameter Integer nSeg=borFie.nSeg;

  Real Nu_ibpsa = BuildingMpc.Fluid.Geothermal.Borefields.BaseClasses.convectionResistanceCircularPipe(
  hSeg=iNFRAX_bF.conDat.hBor  /nSeg,
  rTub=iNFRAX_bF.conDat.rTub,
  eTub=iNFRAX_bF.conDat.eTub,
  kMed = kMed,
  cpMed = cpMed,
  muMed = muMed,
  m_flow = ramp.y,
  m_flow_nominal=iNFRAX_bF.conDat.mBor_flow_nominal);

    parameter Modelica.SIunits.SpecificHeatCapacity cpMed=
      Medium.specificHeatCapacityCp(Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Specific heat capacity of the fluid";
  parameter Modelica.SIunits.ThermalConductivity kMed=
      Medium.thermalConductivity(Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Thermal conductivity of the fluid";
  parameter Modelica.SIunits.DynamicViscosity muMed=Medium.dynamicViscosity(
      Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Dynamic viscosity of the fluid";
  TwoUTube twoUTube(
    redeclare package Medium = Medium,
    borFieDat=iNFRAX_bF,
    show_T=true,
    r_b=6,
    TExt0_start=(273.15 + 13.5))
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{46,10},{66,30}})));
public
  parameter INFRAX.Data.Parameters.BorefieldData.INFRAX_bF            iNFRAX_bF "Borefield data"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{-50,10},{-30,30}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutCon(
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{10,10},{30,30}})));
  IDEAS.Fluid.Sources.Boundary_pT sou(
    nPorts=1,
    redeclare package Medium = Medium,
    use_T_in=true,
    T=280.65)
    annotation (Placement(transformation(extent={{-80,10},{-60,30}})));
  IDEAS.Fluid.Sources.Boundary_pT sin(nPorts=1, redeclare package Medium =
        Medium) annotation (Placement(transformation(extent={{100,10},{80,30}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow1(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{46,-68},{66,-48}})));
public
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn1(
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{-50,-68},{-30,-48}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutSim(
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{10,-68},{30,-48}})));
  IDEAS.Fluid.Sources.Boundary_pT sou1(
    nPorts=1,
    redeclare package Medium = Medium,
    use_T_in=true,
    T=280.65)
    annotation (Placement(transformation(extent={{-80,-68},{-60,-48}})));
  IDEAS.Fluid.Sources.Boundary_pT sin1(nPorts=1, redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{100,-68},{80,-48}})));
  IBPSA.Fluid.Geothermal.Borefields.TwoUTubes borFie(
    redeclare package Medium = Medium,
    borFieDat=iNFRAX_bF,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true,
    dT_dz=0,
    r={6},
    nbTem=1,
    TExt0_start=286.65) "Borehole connected to a discrete ground model"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-10,-58})));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable2D(
    tableOnFile=true,
    tableName="data",
    fileName=ModelicaServices.ExternalReferences.loadResource("modelica://INFRAX/Resources/Validation/borefield_validation.txt"),
    columns={2,3,4,5,6,7})
    annotation (Placement(transformation(extent={{-122,78},{-102,98}})));

  IDEAS.Utilities.Time.ModelTime modTim
    annotation (Placement(transformation(extent={{-164,78},{-144,98}})));
  Modelica.Blocks.Sources.Constant cToK(k=273.15)
    annotation (Placement(transformation(extent={{-90,66},{-78,78}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-70,80},{-50,100}})));
  Modelica.Blocks.Math.Gain        cToK1(k=10/3600*Medium.d_const/1000)
    annotation (Placement(transformation(extent={{-32,38},{-20,50}})));
  Modelica.Blocks.Math.Gain        cToK2(k=10/3600*Medium.d_const/1000)
    annotation (Placement(transformation(extent={{-28,-30},{-16,-18}})));
  Modelica.Blocks.Sources.Ramp
                            ramp(duration=30000, height=iNFRAX_bF.conDat.mBor_flow_nominal
        *2)
    annotation (Placement(transformation(extent={{30,56},{50,76}})));
equation
  connect(sou.ports[1], TIn.port_a)
    annotation (Line(points={{-60,20},{-50,20}}, color={0,127,255}));
  connect(sin.ports[1], mFlow.port_b)
    annotation (Line(points={{80,20},{66,20}}, color={0,127,255}));
  connect(mFlow.port_a, TOutCon.port_b)
    annotation (Line(points={{46,20},{30,20}}, color={0,127,255}));
  connect(TOutCon.port_a,twoUTube. port_b)
    annotation (Line(points={{10,20},{0,20}}, color={0,127,255}));
  connect(twoUTube.port_a, TIn.port_b)
    annotation (Line(points={{-20,20},{-30,20}}, color={0,127,255}));
  connect(sou1.ports[1], TIn1.port_a)
    annotation (Line(points={{-60,-58},{-50,-58}}, color={0,127,255}));
  connect(sin1.ports[1], mFlow1.port_b)
    annotation (Line(points={{80,-58},{66,-58}}, color={0,127,255}));
  connect(mFlow1.port_a, TOutSim.port_b)
    annotation (Line(points={{46,-58},{30,-58}}, color={0,127,255}));
  connect(TIn1.port_b, borFie.port_a)
    annotation (Line(points={{-30,-58},{-20,-58}}, color={0,127,255}));
  connect(borFie.port_b, TOutSim.port_a)
    annotation (Line(points={{0,-58},{10,-58}}, color={0,127,255}));
  connect(modTim.y,combiTable2D. u)
    annotation (Line(points={{-143,88},{-124,88}}, color={0,0,127}));
  connect(combiTable2D.y[2],add. u1) annotation (Line(points={{-101,88},{-96,88},
          {-96,96},{-72,96}},
                        color={0,0,127}));
  connect(cToK.y,add. u2) annotation (Line(points={{-77.4,72},{-76,72},{-76,84},
          {-72,84}}, color={0,0,127}));
  connect(add.y, sou.T_in) annotation (Line(points={{-49,90},{-42,90},{-42,48},{
          -102,48},{-102,24},{-82,24}}, color={0,0,127}));
  connect(add.y, sou1.T_in) annotation (Line(points={{-49,90},{-42,90},{-42,48},
          {-102,48},{-102,-54},{-82,-54}}, color={0,0,127}));
  connect(combiTable2D.y[3], cToK1.u) annotation (Line(points={{-101,88},{-94,88},
          {-94,44},{-33.2,44}}, color={0,0,127}));
  connect(cToK2.u, combiTable2D.y[3]) annotation (Line(points={{-29.2,-24},{-58,
          -24},{-58,-26},{-96,-26},{-96,88},{-101,88}}, color={0,0,127}));
  connect(cToK2.y, mFlow1.m_flow_in)
    annotation (Line(points={{-15.4,-24},{50,-24},{50,-50}}, color={0,0,127}));
  connect(cToK1.y, mFlow.m_flow_in) annotation (Line(points={{-19.4,44},{14,44},
          {14,46},{50,46},{50,28}}, color={0,0,127}));
  annotation (
    experiment(
      StartTime=480,
      StopTime=17365920,
      Interval=480,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput(events=false),
    __Dymola_experimentFlags(
      Advanced(
        EvaluateAlsoTop=false,
        GenerateVariableDependencies=false,
        OutputModelicaCode=false,
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001),
      Evaluate=true,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end InfraxData;
