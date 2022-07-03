//@ts-check

export default class Video{
    /**
     * @param {any} elem
     * @param {import("./status.mjs").default} status
     */
    constructor(elem, status) {
        this.elem = elem;
        this.status = status;

        this.status.error("No video source configured!");
    }
}