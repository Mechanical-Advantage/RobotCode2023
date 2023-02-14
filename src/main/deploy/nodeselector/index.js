// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const robotToDashboardTopic = "/nodeselector/robot_to_dashboard";
const dashboardToRobotTopic = "/nodeselector/dashboard_to_robot";

function setActive(index) {
  Array.from(document.getElementsByClassName("active")).forEach((element) => {
    element.classList.remove("active");
  });
  if (index !== null) {
    document.getElementsByTagName("td")[index].classList.add("active");
  }
}

let client = new NT4_Client(
  window.location.hostname,
  "NodeSelector",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === robotToDashboardTopic) {
      document.body.style.backgroundColor = "white";
      setActive(value);
    }
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    setActive(null);
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe([robotToDashboardTopic], false, false, 0.02);
  client.publishTopic(dashboardToRobotTopic, "int");
  client.connect();

  // Add click listeners
  Array.from(document.getElementsByTagName("td")).forEach((cell, index) => {
    cell.addEventListener("click", () => {
      client.addSample(dashboardToRobotTopic, index);
    });
    cell.addEventListener("contextmenu", (event) => {
      event.preventDefault();
      client.addSample(dashboardToRobotTopic, index);
    });
  });

  // Add touch listeners
  ["touchstart", "touchmove"].forEach((eventString) => {
    document.body.addEventListener(eventString, (event) => {
      event.preventDefault();
      if (event.touches.length > 0) {
        let x = event.touches[0].clientX;
        let y = event.touches[0].clientY;
        Array.from(document.getElementsByTagName("td")).forEach(
          (cell, index) => {
            let rect = cell.getBoundingClientRect();
            if (
              x >= rect.left &&
              x <= rect.right &&
              y >= rect.top &&
              y <= rect.bottom
            ) {
              client.addSample(dashboardToRobotTopic, index);
            }
          }
        );
      }
    });
  });
});
