import { ExtensionContext } from "@foxglove/extension";

import { initPanel } from "./Panel";

export function activate(context: ExtensionContext): void {
  context.registerPanel({ name: "Panthera Collection", initPanel });
}
