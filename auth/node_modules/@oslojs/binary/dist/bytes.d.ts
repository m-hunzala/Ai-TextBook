export declare function compareBytes(a: Uint8Array, b: Uint8Array): boolean;
export declare function concatenateBytes(a: Uint8Array, b: Uint8Array): Uint8Array;
export declare function concatenateBytesInto(target: Uint8Array, a: Uint8Array, b: Uint8Array): void;
export declare class DynamicBuffer {
    private value;
    capacity: number;
    length: number;
    constructor(capacity: number);
    write(bytes: Uint8Array): void;
    writeByte(byte: number): void;
    readInto(target: Uint8Array): void;
    bytes(): Uint8Array;
    clear(): void;
}
