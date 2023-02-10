import { NT4_Client } from "./NT4.js";

const robotToDashboardTopic = "/nodeselector/robot_to_dashboard";
const dashboardToRobotTopic = "/nodeselector/dashboard_to_robot";
const allianceTopic = "/FMSInfo/IsRedAlliance";

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
      Array.from(document.getElementsByClassName("active")).forEach(
        (element) => {
          element.classList.remove("active");
        }
      );
      document.getElementsByTagName("td")[value].classList.add("active");
    } else if (topic.name == allianceTopic) {
      let isRed = value;
      Array.from(document.getElementsByClassName("cone-alliance")).forEach(
        (element) => {
          element.classList.remove(isRed ? "cone-blue" : "cone-red");
          element.classList.add(isRed ? "cone-red" : "cone-blue");
        }
      );
    }
  },
  () => {
    // Connected
    document.body.style.backgroundColor = "white";
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    Array.from(document.getElementsByClassName("active")).forEach((element) => {
      element.classList.remove("active");
    });
    Array.from(document.getElementsByClassName("cone-alliance")).forEach(
      (element) => {
        element.classList.remove("cone-red");
        element.classList.remove("cone-blue");
      }
    );
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe([robotToDashboardTopic], false, false, 0.02);
  client.subscribe([allianceTopic], false, false, 0.25);
  client.publishTopic(dashboardToRobotTopic, "int");
  client.connect();

  // Add click listeners
  Array.from(document.getElementsByTagName("td")).forEach((cell, index) => {
    cell.addEventListener("click", () => {
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
