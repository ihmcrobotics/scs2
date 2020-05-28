const NAMESPACE_SEPARATOR_CHARACTER = ".";
const NAMESPACE_SEPARATOR_REGEX = /\./gi;
const ILLEGAL_NAME_CHARACTERS = "[ .*?@#$%/^&()<>,:{}'\"\\]";
const ILLEGAL_NAME_REGEX = /ILLEGAL_NAME_CHARACTERS/gi;

export function checkForIllegalCharacters(name: string): string {
  if (name.match(ILLEGAL_NAME_REGEX)) {
    console.error(
      "Name cannot contain '${ILLEGAL_NAME_CHARACTERS}'. encountered with: ${name}. Replacing with '_'."
    );
    return name.replace(ILLEGAL_NAME_REGEX, "_");
  }
  else {
    return name;
  }
}

export class NameSpace {
  name: string;
  subNames: string[];

  constructor(fullname: string)
  constructor(namespace: string[])
  constructor(namespace?: string[] | string) {
    if (namespace instanceof Array) {
      this.name = joinNames(namespace);
      this.subNames = namespace;
    }
    else {
      this.name = namespace;
      this.subNames = namespace.split(NAMESPACE_SEPARATOR_CHARACTER);
    }
  }

  subSpace(subName: string): NameSpace {
    let subSpace = new Array(this.subNames.length);
    subSpace.push(this.subNames);
    subSpace.push(subName);
    return new NameSpace(subSpace);
  }

  toString(): string {
    return this.name;
  }
}

function joinNames(subNames: string[]): string {
  for (let i = 0; i < subNames.length; i++) {
    subNames[i] = checkForIllegalCharacters(subNames[i]);
  }
  return subNames.join('.');
}

export class YoVariableRegistry {
  name: string;
  nameSpace: NameSpace;
  yoVariables: YoVariable[];
  parent: YoVariableRegistry;
  children: YoVariableRegistry[];

  constructor(name: string) {
    this.name = checkForIllegalCharacters(name);
    this.nameSpace = new NameSpace(name);
    this.yoVariables = [];
    this.parent = null;
    this.children = [];
  }

  addChild(childRegistry: YoVariableRegistry) {
    this.children.forEach(child => {
      if (child.name === childRegistry.name) {
        console.error("Name collision between registries. This registry name-space: ${this.nameSpace}, child name with name collision: ${childRegistry.name}.");
        return;
      }
    });

    childRegistry.parent = this;
    this.children.push(childRegistry);
  }

  addYoVariable(newVariable: YoVariable) {
    this.yoVariables.forEach(childVariable => {
      if (childVariable.name === newVariable.name) {
        console.error("Name collision between yoVariables. This registry name-space: ${this.nameSpace}, variable name with name collision: ${newVariable.name}.");
        return;
      }
    });

    this.yoVariables.push(newVariable);
  }

  getSubtreeRegistries(): YoVariableRegistry[];

  getSubtreeRegistries(registriesToPack: YoVariableRegistry[]): YoVariableRegistry[];

  getSubtreeRegistries(registriesToPack?: YoVariableRegistry[]): YoVariableRegistry[] {
    if (typeof registriesToPack === 'undefined') {
      registriesToPack = new Array();
    }

    registriesToPack.push(this);
    this.children.forEach(child => child.getSubtreeRegistries(registriesToPack));
    return registriesToPack;
  }

  getSubtreeYoVariables(): YoVariable[];

  getSubtreeYoVariables(yoVariablesToPack: YoVariable[]): YoVariable[];

  getSubtreeYoVariables(yoVariablesToPack?: YoVariable[]): YoVariable[] {
    if (typeof yoVariablesToPack === 'undefined') {
      yoVariablesToPack = new Array();
    }

    this.yoVariables.forEach(yoVariable => yoVariablesToPack.push(yoVariable));
    this.children.forEach(child => child.getSubtreeYoVariables(yoVariablesToPack));
    return yoVariablesToPack;
  }

  toString(): string {
    return this.nameSpace.toString();
  }
}

export abstract class YoVariable {
  name: string;
  nameSpace: NameSpace;
  parent: YoVariableRegistry;

  constructor(name: string, registry: YoVariableRegistry) {
    this.name = checkForIllegalCharacters(name);
    this.nameSpace = registry.nameSpace.subSpace(name);
    this.parent = registry;

    registry.yoVariables.push(this);
  }

  abstract getValue(): any;
}

export class YoEnum extends YoVariable {
  constants: string[];
  ordinal: number;

  constructor(name: string, constants: string[], registry: YoVariableRegistry) {
    super(name, registry);
    this.constants = constants;
    this.ordinal = 0;
  }

  getValue(): string { return this.constants[this.ordinal]; };
}

export class YoBoolean extends YoVariable {
  value: boolean;

  getValue(): boolean { return this.value; }
}

export class YoNumber extends YoVariable {
  value: number;

  getValue(): number { return this.value; }
}

export class YoDouble extends YoNumber {
}

export class YoInteger extends YoNumber {
}

export class YoLong extends YoNumber {
}
