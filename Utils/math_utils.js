// utils
const clamp = (num, min, max) => Math.min(Math.max(num, min), max);
const sum_abs = (array) => array.map(Math.abs).reduce((partialSum, a) => partialSum + a, 0);
const polar = (x, y) => [Math.hypot(x, y), Math.atan2(y, x)];

function normalize_angle(theta) {
    while (theta > Math.PI) {
        theta -= 2 * Math.PI;
    }
    while (theta < -Math.PI) {
        theta += 2 * Math.PI;
    }
    return theta;
}

function arrayEquals(a, b) {
    return Array.isArray(a) &&
        Array.isArray(b) &&
        a.length === b.length &&
        a.every((val, index) => val === b[index]);
}

// like numpy.arange
function arange(start, stop, step, endpoint = false) {
    var num = Math.ceil(Math.abs(start - stop) / Math.abs(step));
    arr = Array.from({ length: num }, (_, i) => start + step * i);
    if (endpoint && Math.abs(arr[arr.length-1] - stop) > (step/2.0) ) {
        arr.push(stop);
    }
    return arr;
}

function linspace(start, stop, num, endpoint = false) {
    const div = endpoint ? (num - 1) : num;
    const step = (stop - start) / div;
    return Array.from({length: num}, (_, i) => start + step * i);
}