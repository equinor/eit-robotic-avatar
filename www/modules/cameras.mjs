//@ts-check
const leftCam = "f0914100f10755b1b9414d0b890e88ca9e14caec37cbe17380b782254f87d162";
const leftTag = "#lefttag"
const rightCam = "2c2de863e9ae1abcc37d6cdecf5cdbfbd27b5d7ef916b7545ffef22cf72c8efa";
const rightTag = "#righttag"

/**
 * @return {Promise<{left:MediaStream, right:MediaStream}>}
 */
export async function loadCams(){
    // Don't use await her as i want load cams in parallel.
    let left = loadWebCam(leftCam, leftTag);
    let right = loadWebCam(rightCam, rightTag);

    // Awaiting bot cams
    let cams = {
        left: await left,
        right: await right,
    }

    // log list of devices to console.
    let devices = await navigator.mediaDevices.enumerateDevices();
    devices.forEach(function(device) {
        console.log(device.kind + ": " + device.label + " id = " + device.deviceId);
    });

    return cams;
}

export function loadRtc(streams){
    loadStream(streams.left, leftTag);
    loadStream(streams.right, rightTag);
}

/* ---- Private stuff --- */

/**
 * @param {string} cam
 * @param {string} tag
 * @return {Promise<MediaStream>}
 */
 async function loadWebCam(cam, tag) {
    let stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: cam } });
    loadStream(stream, tag)
    return stream;
}

/**
 * @param {MediaStream} stream
 * @param {string} tag
 */
 function loadStream(stream, tag) {
    console.log(stream, tag);
    let video = document.querySelector(tag);
    // @ts-ignore
    video.srcObject = stream;
}