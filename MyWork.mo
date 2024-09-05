within ;
package MyWork "My description"

  package Tutorial1

    model SimplePendulumUnits
      constant Real g(unit="m/s2") = 9.81;
      parameter Real L(min=0,unit="m") = 1;
      Real Theta(start=0.1, fixed=true);
      Real ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=50000));
    end SimplePendulumUnits;

    model SimplePendulumSI
      constant Modelica.Units.SI.Acceleration g = 9.81;
      parameter Modelica.Units.SI.Length L(min=0) = 1;
      Modelica.Units.SI.Angle Theta(start=0.1, fixed=true);
      Modelica.Units.SI.AngularVelocity ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=50000));
    end SimplePendulumSI;

    model SimplePendulumImport
      import Modelica.Units.SI;
      constant SI.Acceleration g = 9.81;
      parameter SI.Length L(min=0) = 1;
      SI.Angle Theta(start=0.1, fixed=true);
      SI.AngularVelocity ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=50000));
    end SimplePendulumImport;

    model SimplePendulumDescription "Model of a simple pendulum"
      import Modelica.Units.SI "Import to allow compact names";
      constant SI.Acceleration g=9.81 "Gravitational constant";
      parameter SI.Length L(min=0) = 1 "Length of the pendulum";
      /* List of variables */

      SI.Angle Theta(start=0.1, fixed=true) "Displacement angle";
      SI.AngularVelocity ThetaDot "Displacement velocity";
    equation
      ThetaDot = der(Theta) "Equation to allow second derivative";
      der(ThetaDot) = -g/L*sin(Theta);
    end SimplePendulumDescription;

    model SimplePendulum
      constant Real g = 9.81;
      parameter Real L = 1;
      Real Theta(start=0.1, fixed=true);
      Real ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
    end SimplePendulum;
  end Tutorial1;
  annotation (uses(Modelica(version="4.0.0")));
end MyWork;
