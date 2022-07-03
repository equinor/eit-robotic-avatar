import RoboticAvatar from './src/app.mjs'

async function main() {
    if (document.readyState == "loading" ) {
        // The browser is still building the DOM from the HTML file. We need to wait.
        await new Promise(function(resolve) {
            document.addEventListener('DOMContentLoaded', (event) => {
                resolve();
            });
        })
    }
    console.log("Starting Robot Avatar");

    const app_elem = document.getElementById("robotic_avatar");
    const app = new RoboticAvatar(app_elem);

    // add app to windows for easier debugging
    window.robotic_avatar = app;
}

await main();