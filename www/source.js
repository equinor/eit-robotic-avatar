const leftCam = "ff95b7b8a4a8b9a3653a532f33846865ea2084b8e35994d302e7775c57a30ce3";
const leftTag = "#lefttag"
const rightCam = "ef152f177f126831e759d9bb49c59a8ce13d2f958746d90bb3b940b426cf9bf1";
const rightTag = "#righttag"

async function loadWebCam(cam, tag) {
    let video = document.querySelector(tag);
    let stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: cam } });
    video.srcObject = stream;
    return stream;
}


async function async_main() {
    let peer = new RTCPeerConnection();

    let left = await loadWebCam(leftCam, leftTag);
    for (const track of left.getTracks()) {
        peer.addTrack(track, left);
    }

    let right = await loadWebCam(rightCam, rightTag);
    for (const track of right.getTracks()) {
        peer.addTrack(track, right);
    }

    let devices = await navigator.mediaDevices.enumerateDevices();
    devices.forEach(function(device) {
        console.log(device.kind + ": " + device.label + " id = " + device.deviceId);
    });

    let offer = await peer.createOffer();
    peer.setLocalDescription(offer);
    console.log(offer);
    console.log((JSON.stringify(offer.toJSON())));
}

function main() {
    async_main().then(function() {
        console.log("Main Done");
    }).catch(function(err) {
        console.log(err.name + ": " + err.message);
    });
}

main();