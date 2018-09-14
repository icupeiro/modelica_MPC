within BuildingMpc.Fluid.Geothermal.Borefields.Development;
model MultipleBorehole "multiple borehole model for MPC"

  replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
      annotation (choicesAllMatching = true);

  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));

  parameter
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
    borFieDat "Borefield parameters";

  SingleBorehole singleBorehole(
    redeclare package Medium = Medium,
    soilTemp=soilTemp,
    borFieDat=borFieDat,
    dp_nominal=dp_nominal,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.MassFlowRateMultiplier
                                     masFloDiv(
    redeclare package Medium = Medium,
  allowFlowReversal=false,
    k=borFieDat.conDat.nbBor)
                             "Division of flow rate"
    annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.MassFlowRateMultiplier
                                     masFloMul(
    redeclare package Medium = Medium,
  allowFlowReversal=false,
    k=borFieDat.conDat.nbBor)
                             "Mass flow multiplier"
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  parameter Modelica.SIunits.Temperature soilTemp=273.15 + 10.8
    "Undisturbed temperature of the ground";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Nominal mass flow rate through the borehole";
  parameter Modelica.SIunits.PressureDifference dp_nominal
    "Pressure difference through the borehole";
equation

  assert(borFieDat.conDat.nbBor >= 1, "incorrect amount of boreholes for the borefield");

  connect(port_a, masFloDiv.port_b)
    annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
  connect(masFloDiv.port_a, singleBorehole.port_a) annotation (Line(points={{-60,
          0},{-36,0},{-36,0},{-10,0}}, color={0,127,255}));
  connect(singleBorehole.port_b, masFloMul.port_a)
    annotation (Line(points={{10,0},{60,0}}, color={0,127,255}));
  connect(masFloMul.port_b, port_b)
    annotation (Line(points={{80,0},{100,0}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
end MultipleBorehole;
