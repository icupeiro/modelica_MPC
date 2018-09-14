within BuildingMpc.Fluid.Geothermal.Borefields.Linear.Buses;
expandable connector InputBus
  extends Modelica.Icons.SignalBus;

  //TABS heat input
  input Modelica.SIunits.Temperature TBorIn( start=273.15+13.5);
  input Modelica.SIunits.MassFlowRate mFlow( start=0.3);

  annotation (
    defaultComponentName="weaBus",
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-0,-0},{0,
            0}}), graphics={Rectangle(
          extent={{-20,2},{22,-2}},
          lineColor={255,204,51},
          lineThickness=0.5)}),
    Documentation(info="<html>
<p>
This component is an expandable connector that is used to implement a bus that contains the weather data.
</p>
</html>", revisions="<html>
<ul>
<li>
June 25, 2010, by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"));
end InputBus;
