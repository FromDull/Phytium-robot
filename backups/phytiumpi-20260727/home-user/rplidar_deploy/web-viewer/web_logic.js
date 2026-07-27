(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.WebViewerLogic = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const destinationAliases = [
    ["笔记本电脑", "laptop"],
    ["笔记本", "laptop"],
    ["电脑", "laptop"],
    ["长椅", "bench"],
    ["椅子", "chair"],
    ["凳子", "chair"],
    ["餐桌", "dining-table"],
    ["桌子", "dining-table"],
    ["杯子", "cup"],
    ["水杯", "cup"],
    ["电视机", "tv"],
    ["电视", "tv"],
    ["手机", "cell-phone"],
    ["电话", "cell-phone"],
    ["人员", "person"],
    ["人物", "person"],
    ["人", "person"],
  ];

  function finite(value, fallback = 0) {
    const number = Number(value);
    return Number.isFinite(number) ? number : fallback;
  }

  function negated(value) {
    const result = -finite(value);
    return Object.is(result, -0) ? 0 : result;
  }

  function mirrorGridX(width, gridX) {
    return finite(width) - finite(gridX);
  }

  function displayWorldPoint(point) {
    return {
      x: negated(point?.x),
      y: finite(point?.y),
      z: finite(point?.z),
    };
  }

  function displayWorldQuaternion(quaternion) {
    return {
      x: finite(quaternion?.x),
      y: negated(quaternion?.y),
      z: negated(quaternion?.z),
      w: finite(quaternion?.w, 1),
    };
  }

  function normalizeText(value) {
    return String(value || "")
      .trim()
      .toLowerCase()
      .replace(/[，。！？、；：,!?;:]/g, " ")
      .replace(/\s+/g, " ");
  }

  function canonicalDestinationLabel(destination) {
    const normalized = normalizeText(destination).replace(/\s/g, "");
    for (const [alias, label] of destinationAliases) {
      if (normalized.includes(alias)) return label;
    }
    return normalizeText(destination).replace(/\s+/g, "-");
  }

  function parseCoordinateGoal(destination) {
    const text = normalizeText(destination);
    let match = text.match(/x\s*=?\s*(-?\d+(?:\.\d+)?)\s*(?:米)?\s*y\s*=?\s*(-?\d+(?:\.\d+)?)/i);
    if (!match) {
      match = text.match(/(?:坐标|位置)?\s*(-?\d+(?:\.\d+)?)\s*(?:米)?\s+(-?\d+(?:\.\d+)?)\s*(?:米)?/);
    }
    if (!match) return null;
    const x = Number(match[1]), y = Number(match[2]);
    return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
  }

  function chooseLandmarkGoal(destination, objects, robotPosition, clearanceM = 0.6) {
    const label = canonicalDestinationLabel(destination);
    const candidates = (objects || []).filter((object) =>
      object?.confirmed && String(object.label || "").toLowerCase().replace(/\s+/g, "-") === label
    );
    if (!candidates.length) return null;
    const robot = robotPosition && Number.isFinite(Number(robotPosition.x)) && Number.isFinite(Number(robotPosition.y))
      ? { x: Number(robotPosition.x), y: Number(robotPosition.y) }
      : null;
    candidates.sort((left, right) => {
      if (robot) {
        const leftDistance = Math.hypot(Number(left.position?.x) - robot.x, Number(left.position?.y) - robot.y);
        const rightDistance = Math.hypot(Number(right.position?.x) - robot.x, Number(right.position?.y) - robot.y);
        if (leftDistance !== rightDistance) return leftDistance - rightDistance;
      }
      return Number(right.last_seen || 0) - Number(left.last_seen || 0);
    });
    const landmark = candidates[0], position = {
      x: Number(landmark.position?.x),
      y: Number(landmark.position?.y),
    };
    if (!Number.isFinite(position.x) || !Number.isFinite(position.y)) return null;
    let goal = { ...position };
    if (robot) {
      const dx = position.x - robot.x, dy = position.y - robot.y, distance = Math.hypot(dx, dy);
      if (distance > clearanceM + 0.2) {
        goal = {
          x: position.x - dx / distance * clearanceM,
          y: position.y - dy / distance * clearanceM,
        };
      }
    }
    return {
      goal,
      landmark_position: position,
      object_id: landmark.id,
      label: landmark.label,
    };
  }

  return {
    canonicalDestinationLabel,
    chooseLandmarkGoal,
    displayWorldPoint,
    displayWorldQuaternion,
    mirrorGridX,
    parseCoordinateGoal,
  };
});
