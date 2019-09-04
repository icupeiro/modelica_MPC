within BuildingMpc.Examples.ControllerModels.Case900Paper;
model Case900SouStaUpd
  extends Case900Sou(source(use_T_in=true));
  Modelica.Blocks.Sources.RealExpression T_source(y=heatCapacitor.T)
    annotation (Placement(transformation(extent={{-78,58},{-58,78}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=
        Modelica.Constants.inf, T(fixed=true, start=278.15))
    annotation (Placement(transformation(extent={{-102,54},{-82,74}})));
equation
  connect(source.T_in, T_source.y) annotation (Line(points={{-48,74},{-54,74},{-54,
          68},{-57,68}}, color={0,0,127}));
end Case900SouStaUpd;
