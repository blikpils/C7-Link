# C7 Link — Bowflex C7 Smart Bike Bridge


C7 Link is an ESP32-based project that connects directly to the Bowflex C7's internal logic board, capturing its sensor data and bridging it to modern fitness devices over Bluetooth.

Why C7 Link?

The Bowflex C7's original Android console limits the riding experience, offering little functionality for guest users while pushing the JRNY subscription ecosystem. Slow boot times and promotional content add unnecessary friction when all you want to do is ride.

C7 Link removes that dependency by connecting directly to the bike's internal electronics, allowing it to be used without relying on the original console. It brings workout data to your iPhone and Apple Watch over Bluetooth, opening the door to a more flexible setup where you can track your rides and get your data into platforms like Strava without being tied to JRNY.

The idea is simple: you already own the bike, so you should be free to use it your way.

### Current capabilities
- **Direct hardware communication:** Established communication between an ESP32-C3 and the C7's logic board.
- **Protocol initialization:** Identified the command sequence required to initialize data transmission from the bike.
- **Raw data capture:** Captured serial data while varying resistance, cadence, and speed to investigate how workout metrics are encoded.
- **Bluetooth connectivity:** Successfully connected the ESP32 to an Apple Watch and iPhone for integration with their fitness ecosystem.
- **Protocol analysis:** Reverse-engineering the bike's serial data to identify and decode individual workout metrics.

### In development
- **Metric decoding:** Refining the interpretation and scaling of resistance, cadence, speed, distance, and other available values.
- **Data accuracy:** Validating decoded metrics against actual riding conditions.
- **Further integration:** Improving the reliability and completeness of the data exposed to compatible fitness devices and applications.

The goal is to unlock the data already available inside the Bowflex C7, overcome the limitations of its original console, and make the bike compatible with modern fitness technology—all without replacing its internal electronics.

**C7 Link — Your bike. Your data. Connected.**
