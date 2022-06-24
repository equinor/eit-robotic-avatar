const leftTag = "#lefttag"
const rightTag = "#righttag"

async function loadWebCam(cam, tag) {
    let video = document.querySelector(tag);
    let stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: cam } });
    video.srcObject = stream;
    return stream;
}

let peer = new RTCPeerConnection();

async function async_main() {

}

function main() {
    async_main().then(function() {
        console.log("Main Done");
    }).catch(function(err) {
        console.log(err.name + ": " + err.message);
    });
}

main();

async function rtc_offer_async(offer) {
    await peer.setRemoteDescription(offer);
    let awnser = await peer.createAnswer();
    peer.setLocalDescription(awnser);
    
    console.log(awnser);
    console.log((JSON.stringify(awnser.toJSON())));
}

function rtc_offer(offer) {
    rtc_offer_async(offer).then(function() {
        console.log("Offer Done");
    }).catch(function(err) {
        console.log(err.name + ": " + err.message);
    });
}