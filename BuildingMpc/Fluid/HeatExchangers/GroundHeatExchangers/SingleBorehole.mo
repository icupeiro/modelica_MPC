within BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers;
model SingleBorehole "single borehole model for MPC"

  replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
      annotation (choicesAllMatching = true);

  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.BoreHoles.SingleBoreHoleUTube
    borehole(
    allowFlowReversal=false,
    computeFlowResistance=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    borFieDat=borFieDat,
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    intHex(RVol1(each y = BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.convectionResistance(hSeg=borFieDat.conDat.hSeg, rBor=borFieDat.conDat.rBor, rTub=borFieDat.conDat.rTub, eTub=borFieDat.conDat.eTub, kMed=kMed, mueMed=mueMed, cpMed=cpMed, m_flow_nominal=m_flow_nominal)),
    RVol2(each y = BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.convectionResistance(hSeg=borFieDat.conDat.hSeg, rBor=borFieDat.conDat.rBor, rTub=borFieDat.conDat.rTub, eTub=borFieDat.conDat.eTub, kMed=kMed, mueMed=mueMed, cpMed=cpMed, m_flow_nominal=m_flow_nominal))),
    m_flow_small=1E-2*abs(m_flow_nominal))
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,0})));
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer
    lay[borFieDat.conDat.nVer](
    each soiDat=borFieDat.soiDat,
    each h=borFieDat.conDat.hSeg,
    each r_a=borFieDat.conDat.rBor,
    each r_b=borFieDat.conDat.rExt,
    each nSta=borFieDat.conDat.nHor,
    each TInt_start=borFieDat.conDat.T_start,
    each TExt_start=borFieDat.conDat.T_start) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature[borFieDat.conDat.nVer]
    prescribedTemperature
    annotation (Placement(transformation(extent={{40,50},{20,70}})));
  Modelica.Blocks.Sources.Constant[borFieDat.conDat.nVer] tempProfile(k=
        soilTemp)
    "undisturbed ground temperature, could be included as vertical profile"
    annotation (Placement(transformation(extent={{80,50},{60,70}})));
  parameter Modelica.SIunits.Temperature soilTemp=273.15 + 10.8
    "Undisturbed temperature of the ground";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Nominal mass flow rate through the borehole"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.PressureDifference dp_nominal
    "Pressure difference through the borehole"
    annotation (Dialog(group="Nominal condition"));
  parameter
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
    borFieDat "Borefield parameters";
protected
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
  parameter Modelica.SIunits.DynamicViscosity mueMed=Medium.dynamicViscosity(
      Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Dynamic viscosity of the fluid";

equation
  connect(tempProfile.y, prescribedTemperature.T)
    annotation (Line(points={{59,60},{42,60}}, color={0,0,127}));
  connect(prescribedTemperature.port, lay.port_b)
    annotation (Line(points={{20,60},{0,60},{0,50}}, color={191,0,0}));
  connect(lay.port_a, borehole.port_wall) annotation (Line(points={{-4.44089e-16,
          30},{0,30},{0,10}}, color={191,0,0}));
  connect(port_a, borehole.port_a)
    annotation (Line(points={{-100,0},{-10,0}},         color={0,127,255}));
  connect(borehole.port_b, port_b)
    annotation (Line(points={{10,0},{100,0}},         color={0,127,255}));
                 annotation (Placement(transformation(extent={{-10,-10},{10,10}})),
              Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-70,80},{70,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,-52},{62,-60}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,58},{62,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,6},{62,0}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{54,92},{46,-88}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-54,-88},{-46,92}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-72,80},{-62,-80}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{62,80},{72,-80}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-52,-80},{54,-88}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));

end SingleBorehole;
