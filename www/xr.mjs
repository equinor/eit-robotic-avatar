async function main() {
    let xr = navigator.xr;
    console.log("immersive-ar:", await xr.isSessionSupported("immersive-ar"));
    console.log("immersive-vr:", await xr.isSessionSupported("immersive-vr"));
    console.log("inline", await xr.isSessionSupported("inline"));

    let session = await xr.requestSession("immersive-vr");

    let ref_frame = await session.requestReferenceSpace("local");

    function vrframe(time, frame) {
        console.log(frame.getViewerPose(ref_frame).transform.orientation.toJSON());
        debugger;

        session.requestAnimationFrame(vrframe);
    }
    
    window.vrtest = session;
    window.vr_frame = vrframe;

    
    let glCanvas = document.getElementById("glcanvas");
    let gl = glCanvas.getContext("webgl", { xrCompatible: true });

    session.updateRenderState({
        baseLayer: new XRWebGLLayer(session, gl)
    });

    session.requestAnimationFrame(vrframe);
}

document.getElementById("vr").onclick = main;