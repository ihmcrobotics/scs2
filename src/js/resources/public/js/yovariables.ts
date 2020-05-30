const NAMESPACE_SEPARATOR_CHARACTER = ".";
const NAMESPACE_SEPARATOR_REGEX = /\./gi;
const ILLEGAL_NAME_CHARACTERS = "[ .*?@#$%/^&()<>,:{}'\"\\]";
const ILLEGAL_NAME_REGEX = /ILLEGAL_NAME_CHARACTERS/gi;

export function checkForIllegalCharacters(name: string): string
export function checkForIllegalCharacters(names: string[]): string[]
export function checkForIllegalCharacters(input: string | string[]): string | string[] {
  if (input instanceof Array) {
    for (let i = 0; i < input.length; i++) {
      input[i] = checkForIllegalCharacters(input[i]);
    }
  } else if (input.match(ILLEGAL_NAME_REGEX)) {
    console.error(
      "Name cannot contain '${ILLEGAL_NAME_CHARACTERS}'. encountered with: ${name}. Replacing with '_'."
    );
    input = input.replace(ILLEGAL_NAME_REGEX, "_");
  }

  return input;
}

export function areArraysEqual<T>(array1: Array<T>, array2: Array<T>): boolean {
  if (array1 === undefined) { return array2 === undefined; }
  if (array2 === undefined) { return false; }
  if (array1.length !== array2.length) { return false; }

  for (let i = 0; i < array1.length; i++) {
    let e1 = array1[i];
    let e2 = array2[i];
    // TODO Could not figure out how to get an equals method from T if it exists.
    if (e1 !== e2)
      return false;
  }

  return true;
}

export class NameSpace {
  name: string;
  subNames: string[];

  constructor(namespace: string[] | string) {
    if (namespace instanceof Array) {
      this.name = checkForIllegalCharacters(namespace).join(".");
      this.subNames = namespace;
    } else {
      this.name = namespace;
      this.subNames = namespace.split(NAMESPACE_SEPARATOR_CHARACTER);
    }
  }

  slice(startIndex: number, length?: number): NameSpace {
    if (startIndex < 0) {
      console.error("Invalid argument, startIndex cannot be negative: ${startIndex}.");
      return null;
    }

    if (length === undefined) {
      if (startIndex >= this.length()) {
        console.error("Invalid argument: startIndex (= ${startIndex}) has to be less than this.length (= ${this.subNames.length}).");
        return null;
      }

      return new NameSpace(this.subNames.slice(startIndex, this.length() - startIndex));
    }
    else {
      if (startIndex + length > this.subNames.length) {
        console.error("Invalid arguments: startIndex = ${startIndex}, length = ${length}, this.length = ${this.subNames.length}.");
        return null;
      }

      return new NameSpace(this.subNames.slice(startIndex, startIndex + length));
    }
  }

  startWith(other: NameSpace): boolean {
    if (other.length() > this.length()) { return false; }

    for (let i = 0; i < other.length(); i++) {
      if (this.subNames[i] !== other.subNames[i])
        return false;
    }
  }

  endWith(other: NameSpace): boolean {
    if (other.length() > this.length()) { return false; }

    for (let i = 0; i < other.length(); i++) {
      if (this.subNames[this.length() - i] !== other.subNames[other.length() - i])
        return false;
    }
  }

  stripStart(start: NameSpace): NameSpace {
    if (!this.startWith(start)) { return null; }
    return this.slice(start.length());
  }

  stripEnd(end: NameSpace): NameSpace {
    if (!this.endWith(end)) { return null; }
    return this.slice(0, this.length() - end.length() + 1);
  }

  length(): number { return this.subNames.length; }

  equals(other: NameSpace): boolean {
    return this.name !== other.name && areArraysEqual(this.subNames, other.subNames);
  }

  toString(): string { return this.name; }

  static concatenate(nameSpace1: NameSpace | string, nameSpace2: NameSpace | string): NameSpace {
    let subNames = new Array();
    if (nameSpace1 instanceof NameSpace)
      subNames.push(nameSpace1.subNames);
    else
      subNames.push(new NameSpace(nameSpace1).subNames);

    if (nameSpace2 instanceof NameSpace)
      subNames.push(nameSpace2.subNames);
    else
      subNames.push(new NameSpace(nameSpace2).subNames);

    return new NameSpace(subNames);
  }
}

export class YoRegistry {
  name: string;
  nameSpace: NameSpace;
  yoVariables: YoVariable[];
  parent: YoRegistry;
  children: YoRegistry[];

  constructor(name: string) {
    this.name = checkForIllegalCharacters(name);
    this.nameSpace = new NameSpace(name);
    this.yoVariables = [];
    this.parent = null;
    this.children = [];
  }

  addChild(childRegistry: YoRegistry) {
    this.children.forEach(child => {
      if (child.name === childRegistry.name) {
        console.error("Name collision between registries. This registry name-space: ${this.nameSpace}, child name with name collision: ${childRegistry.name}.");
        return;
      }
    });

    childRegistry.parent = this;
    childRegistry.nameSpace = NameSpace.concatenate(this.nameSpace, childRegistry.name);
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

  getSubtreeYoRegistries(registriesToPack?: YoRegistry[]): YoRegistry[] {
    if (typeof registriesToPack === 'undefined') {
      registriesToPack = new Array();
    }

    registriesToPack.push(this);
    this.children.forEach(child => child.getSubtreeYoRegistries(registriesToPack));
    return registriesToPack;
  }

  getSubtreeYoVariables(yoVariablesToPack?: YoVariable[]): YoVariable[] {
    if (typeof yoVariablesToPack === 'undefined') {
      yoVariablesToPack = new Array();
    }

    this.yoVariables.forEach(yoVariable => yoVariablesToPack.push(yoVariable));
    this.children.forEach(child => child.getSubtreeYoVariables(yoVariablesToPack));
    return yoVariablesToPack;
  }

  isRoot(): boolean {
    return this.parent === null;
  }

  getRoot(): YoRegistry {
    if (this.isRoot())
      return this;
    else
      return this.parent.getRoot();
  }

  getChild(name: string): YoRegistry {
    for (const child of this.children) {
      if (child.name === name)
        return child;
    }
    return null;
  }

  searchYoRegistry(name: string | NameSpace): YoRegistry {
    if (name instanceof NameSpace) {
      if (name.subNames[0] !== this.name) {
        name = name.stripStart(this.nameSpace);
      } else {
        name = name.slice(1);
      }

      if (name === null) { return null; }
      if (name.subNames.length === 0) { return this; }

      let child = this.getChild(name.subNames[0]);
      if (child === null) { return null; }
      return child.searchYoRegistry(name);
    } else {
      // Assume the argument refers only to the name of the registry
      let result = this.getChild(name);
      if (result !== null) { return result; }

      for (const child of this.children) {
        result = child.searchYoRegistry(name);
        if (result !== null)
          return result;
      }

      return null;
    }
  }

  ensureYoRegistryExists(name: NameSpace): YoRegistry {
    let result = this.searchYoRegistry(name);
    if (result !== null) { return result; }

    if (name.subNames[0] !== this.name)
      name = name.stripStart(this.nameSpace);
    else
      name = name.slice(1);

    if (name === null) { return null; }

    let child = this.getChild(name.subNames[0]);

    if (child === null) {
      // TODO Figure out why I need the lousy this() method for the following to work.
      let parent = this.this();
      for (let i = 0; i < name.length(); i++) {
        child = new YoRegistry(name.subNames[0]);
        parent.addChild(child);
        parent = child;
      }
      return child;
    }
    return child.ensureYoRegistryExists(name);
  }

  // TODO Lousy method to cast the keyword 'this' to YoRegistry, really weird.
  private this(): YoRegistry { return this; }

  getYoVariable(name: string): YoVariable {
    for (const yoVariable of this.yoVariables) {
      if (yoVariable.name === name)
        return yoVariable;
    }
    return null;
  }

  searchYoVariable(name: string | NameSpace): YoVariable;
  searchYoVariable(name: string, parentName: string | NameSpace): YoVariable;
  searchYoVariable(name: string | NameSpace, parentName?: string | NameSpace): YoVariable {
    if (parentName === undefined) {
      if (name instanceof NameSpace) {
        return this.searchYoVariable(name.subNames[name.length() - 1], name.slice(0, name.length() - 1));
      } else {
        // Assume the argument refers only to the name of the registry
        let result = this.getYoVariable(name);
        if (result !== null) { return result; }

        for (const child of this.children) {
          result = child.searchYoVariable(name);
          if (result !== null)
            return result;
        }
      }
    } else {
      let parent = this.searchYoRegistry(parentName);
      if (parent === null) { return null; }
      if (name instanceof NameSpace) {
        console.error("Something went wrong, variable name should not a NameSpace when parent name is provided.");
        return null;
      }
      return parent.getYoVariable(name);
    }
  }

  toString(): string {
    return this.nameSpace.toString();
  }
}

export abstract class YoVariable {
  name: string;
  nameSpace: NameSpace;
  parent: YoRegistry;

  constructor(name: string, registry: YoRegistry) {
    this.name = checkForIllegalCharacters(name);
    this.nameSpace = NameSpace.concatenate(registry.nameSpace, name);
    this.parent = registry;

    registry.yoVariables.push(this);
  }

  abstract getValue(): any;

  getFullName(): string {
    return this.nameSpace.toString();
  }
}

export class YoEnum extends YoVariable {
  constants: string[];
  ordinal: number;

  constructor(name: string, constants: string[], registry: YoRegistry) {
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
