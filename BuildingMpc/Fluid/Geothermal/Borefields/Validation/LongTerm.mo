within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model LongTerm
  extends Modelica.Icons.Example;
  package Medium = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=283.15, X_a=0.30);
  parameter Integer nSeg=borFie.nSeg;

  Real Nu_ibpsa = BuildingMpc.Fluid.Geothermal.Borefields.BaseClasses.convectionResistanceCircularPipe(
  hSeg = borFieDat.conDat.hBor/nSeg,
  rTub = borFieDat.conDat.rTub,
  eTub = borFieDat.conDat.eTub,
  kMed = kMed,
  cpMed = cpMed,
  muMed = muMed,
  m_flow = ramp.y,
  m_flow_nominal = borFieDat.conDat.mBor_flow_nominal);

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
  OneUTube oneUTube(
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    borFieDat=borFieDat,
    show_T=true,
    Tsoil=286.65)
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
  parameter IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example  borFieDat(conDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example(nBor=1,
        cooBor={{0,0}}))                                                        "Borefield data"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{-50,10},{-30,30}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutCon(
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
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
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{-50,-68},{-30,-48}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOutSim(
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
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
              IBPSA.Fluid.Geothermal.Borefields.OneUTube                       borFie(
    redeclare package Medium = Medium,
    borFieDat=borFieDat,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true)
    "Borehole connected to a discrete ground model" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-10,-58})));
  Modelica.Blocks.Sources.Sine     sine(
    startTime=0,
    amplitude=borFieDat.conDat.mBor_flow_nominal/2,
    offset=borFieDat.conDat.mBor_flow_nominal/2,
    freqHz=1/1000)
    annotation (Placement(transformation(extent={{-48,-16},{-28,6}})));
  Modelica.Blocks.Math.Gain gain(k=borFieDat.conDat.nBor)
    annotation (Placement(transformation(extent={{28,58},{48,78}})));
  Modelica.Blocks.Sources.Ramp
                            ramp(height=borFieDat.conDat.mBor_flow_nominal*2,
      duration=30000)
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Modelica.Blocks.Sources.Sine     sine1(
    startTime=0,
    amplitude=3,
    offset=273.15 + 5,
    freqHz=1/500)
    annotation (Placement(transformation(extent={{-130,-12},{-110,10}})));
equation
  connect(sou.ports[1], TIn.port_a)
    annotation (Line(points={{-60,20},{-50,20}}, color={0,127,255}));
  connect(sin.ports[1], mFlow.port_b)
    annotation (Line(points={{80,20},{66,20}}, color={0,127,255}));
  connect(mFlow.port_a, TOutCon.port_b)
    annotation (Line(points={{46,20},{30,20}}, color={0,127,255}));
  connect(TOutCon.port_a, oneUTube.port_b)
    annotation (Line(points={{10,20},{0,20}}, color={0,127,255}));
  connect(oneUTube.port_a, TIn.port_b)
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
  connect(gain.y, mFlow.m_flow_in)
    annotation (Line(points={{49,68},{50,68},{50,28}}, color={0,0,127}));
  connect(sine.y, mFlow1.m_flow_in)
    annotation (Line(points={{-27,-5},{50,-5},{50,-50}}, color={0,0,127}));
  connect(sine.y, gain.u) annotation (Line(points={{-27,-5},{4,-5},{4,68},{26,68}},
        color={0,0,127}));
  connect(sine1.y, sou.T_in)
    annotation (Line(points={{-109,-1},{-82,-1},{-82,24}}, color={0,0,127}));
  connect(sine1.y, sou1.T_in)
    annotation (Line(points={{-109,-1},{-82,-1},{-82,-54}}, color={0,0,127}));
  annotation (
    experiment(
      StopTime=30000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(
        GenerateVariableDependencies=false,
        OutputModelicaCode=false,
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001),
      Evaluate=false,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end LongTerm;
