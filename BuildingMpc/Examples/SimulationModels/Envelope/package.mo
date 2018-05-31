within BuildingMpc.Examples.SimulationModels;
package Envelope
  model Bui900TABS
    "model to determine the load of the geothermal borefield"
    extends IDEAS.Buildings.Validation.BaseClasses.Structure.Bui900(
    nEmb=1,
    floor(redeclare parameter BuildingMpc.Examples.Data.HeavyFloorTABS
          constructionType))
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  equation
    connect(floor.port_emb, heatPortEmb) annotation (Line(points={{-10,-14.5},{-6,
            -14.5},{-6,56},{120,56},{120,60},{150,60}}, color={191,0,0}));
  end Bui900TABS;
end Envelope;
