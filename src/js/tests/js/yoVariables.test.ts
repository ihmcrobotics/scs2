import { YoVariableRegistry } from "../../resources/public/js/yovariables";

describe("YoVariableRegistry", function () {
    it("constructor", function () {
        let registry = new YoVariableRegistry("blop");
        expect(registry.name).toBe("blop");
    });
});