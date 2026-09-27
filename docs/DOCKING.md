# Rear copper-contact docking

## What is known

The robot has a forward-facing camera and rear copper contacts with magnets. ArUco docking has not yet been tested. The power source, charge controller and battery protection at the dock have not been identified. Copper contacts and magnets describe the connector, not a lithium-ion charging circuit.

Before any energized docking test, identify and verify the existing charging arrangement with the hardware supervisor. Software cannot make direct battery contact charging safe or provide missing cell balancing/protection. No additional component is assumed to exist.

## Implemented sequence

1. Navigate to a saved, front-facing staging pose using the occupancy map.
2. Stop and require an explicit supervised docking command.
3. Find the configured ArUco marker. Turn toward it and advance slowly to the calibrated stand-off.
4. Require small marker bearing and marker-normal error. If angular geometry is unsuitable, time out instead of assuming alignment.
5. Rotate 180 degrees using gyro yaw integration.
6. Reverse slowly for the configured maximum distance/time. The marker is no longer visible, and the front camera cannot verify rear clearance.
7. Stop immediately when negative charging current appears. Declare charging only after it persists for two seconds. If it disappears during confirmation, resume only within the original distance/time bound.
8. Stop with a fault if contact is not confirmed before the bound.

A charging current measurement supports contact confirmation only if the INA219 is wired to observe net battery current, polarity is calibrated, and the observed threshold is distinguishable from noise. A voltage rise alone does not prove successful docking.

## Prepare the marker

```bash
python tools/marker.py
```

Print `data/dock-marker.png`, with the black square exactly 10 cm wide, excluding its white margin. Default dictionary is DICT_4X4_50, marker ID 0. Mount the marker rigidly at a height and position visible during front alignment. It must remain associated with the contact plane through a measured offset.

## Geometry still required

Measure the axle-to-camera distance, axle-to-rear-contact distance, marker-plane-to-contact-plane distance, and stand-off needed for a clear 180-degree turn. These determine `staging_distance_m` and `reverse_distance_m`. The shipped 0.55 m and 0.25 m values are simulation placeholders, not physical calibration.

The marker alignment algorithm requires an initially reasonable approach direction; it does not implement arbitrary lateral parking manoeuvres. Establish a repeatable staging pose facing the dock. Align it manually first, disarm, and click **Set current pose as dock staging**.

Perform unpowered contact-alignment trials before energized ones. Measure lateral error and contact repeatability, including wheel slip during rotation. The rear must stay clear through the entire test; there is no rear obstacle sensor.

The flags `geometry_calibrated`, `rear_clearance_confirmed`, `charger_verified`, and `sensors.current_sees_charge` intentionally default false. Set them only after physical verification. The UI also asks for supervision for each attempt.

There is no GPIO-controlled charger relay in the confirmed hardware. Reaching a displayed percentage never disconnects charging. Charge completion/protection belongs to the verified charger and battery electronics.
