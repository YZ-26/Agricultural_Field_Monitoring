# Agricultural Field Monitoring System
![Без имени](https://github.com/user-attachments/assets/261b32d9-e7d0-4e56-a887-5a54ec3368af)

## Overview
This project aims to automate agricultural field monitoring to maintain crop health, prevent fires, and manage resources efficiently. Traditional methods are labor-intensive and ineffective for large areas. Our solution employs autonomous balloons and static sensors to gather and transmit environmental data.

## System Components

### Sensors
- **Coverage Area**: 80x80 square meters.
- **Placement**:
  - Sensors are placed to ensure comprehensive coverage and redundancy.
  - Placement algorithm ensures scalability and efficiency in data collection:
    - Two sensors in the high-risk 20x20 meter area (assumed top left corner).
    - Remaining sensors placed in a checkerboard pattern.
- **Data Collection**: Sensors measure temperature every second and publish data to detect fire risks.

### Balloons
- **Function**: Act as data collectors, flying over sensors to gather and transmit data to the base station.
- **Movement**:
  - Movement and data collection strategies are implemented to ensure continuous monitoring:
    - One balloon hovers over the high-risk area.
    - Other balloons patrol remaining sensors.
- **Cache Management**: Data is timestamped and older data is cleared every 5 minutes.

### Base Station
- **Role**: Central controller requesting data from balloons.
- **Request Cycle**: 
  - Sequentially requests data from each sensor every 30 seconds.
  - Retries every 5 seconds if data is not received.
  - Records data delivery time for performance evaluation.

## Evaluation
- **Criteria**: Packet delay, including movement and communication delays.
- **Method**: Measured delivery time for data from sensors to the base station across multiple cycles.
- **Results**: 
  - Average delivery time decreases with fewer sensors.
  - System stabilizes and operates more efficiently after initial adjustments.
  - Effective for real-time monitoring in agricultural fields.

### Test Cases
- Tested with 13, 10, and 7 sensors using 4 balloons.
- Results indicate the system's robustness and efficiency in different configurations.

## Conclusion
Our automated agricultural field monitoring system using sensors and balloons demonstrates an effective and scalable solution for real-time environmental monitoring and fire prevention in agricultural settings. The system's design ensures comprehensive coverage, prompt data collection, and efficient resource management.
