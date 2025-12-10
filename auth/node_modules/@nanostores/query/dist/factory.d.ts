import { PlatformCompat } from './platforms/type';
import { MapStore, ReadableAtom } from 'nanostores';

type NoKey = null | undefined | void | false;
type SomeKey = string | number | true;
export type KeyInput = SomeKey | Array<SomeKey | ReadableAtom<SomeKey | NoKey> | FetcherStore>;
type Key = string;
type KeyParts = SomeKey[];
export type KeySelector = Key | Key[] | ((key: Key) => boolean);
export type Fetcher<T> = (...args: KeyParts) => Promise<T>;
export type OnErrorRetry = (opts: {
    error: unknown;
    key: Key;
    retryCount: number;
}) => number | void | false | null | undefined;
type EventTypes = {
    onError?: (error: unknown) => void;
};
type RefetchSettings = {
    dedupeTime?: number;
    revalidateOnFocus?: boolean;
    revalidateOnReconnect?: boolean;
    revalidateInterval?: number;
    cacheLifetime?: number;
    onErrorRetry?: OnErrorRetry | null | false;
};
export type CommonSettings<T = unknown> = {
    fetcher?: Fetcher<T>;
} & RefetchSettings & EventTypes;
export type NanoqueryArgs = {
    cache?: Map<Key, {
        data?: unknown;
        error?: unknown;
        retryCount?: number;
        created?: number;
        expires?: number;
    }>;
} & CommonSettings;
export type FetcherValue<T = any, E = Error> = {
    data?: T;
    error?: E;
    loading: boolean;
    promise?: Promise<T>;
};
type LazyFetchValue<T = any, E = any> = {
    data: T;
} | {
    error: E;
};
export type FetcherStore<T = any, E = any> = MapStore<FetcherValue<T, E>> & {
    _: Symbol;
    key?: Key;
    invalidate: (...args: any[]) => void;
    revalidate: (...args: any[]) => void;
    mutate: (data?: T) => void;
    fetch: () => Promise<LazyFetchValue<T, E>>;
};
export type FetcherStoreCreator<T = any, E = Error> = (keys: KeyInput, settings?: CommonSettings<T>) => FetcherStore<T, E>;
export type ManualMutator<Data = void, Result = unknown> = (args: {
    data: Data;
    invalidate: (key: KeySelector) => void;
    revalidate: (key: KeySelector) => void;
    getCacheUpdater: <T = unknown>(key: Key, shouldRevalidate?: boolean) => [(newValue?: T) => void, T | undefined];
}) => Promise<Result>;
export type MutateCb<Data, Result = unknown> = Data extends void ? () => Promise<Result> : (data: Data) => Promise<Result>;
export type MutatorStore<Data = void, Result = unknown, E = Error> = MapStore<{
    mutate: MutateCb<Data, Result>;
    data?: Result;
    loading?: boolean;
    error?: E;
}> & {
    mutate: MutateCb<Data, Result>;
};
/**
 * This piece of shenanigans is copy-pasted from SWR. God be my witness I don't like
 * all this bitwise shifting operations as they are absolutely unclear, but I'm
 * ok to compact the code a bit.
 */
export declare function defaultOnErrorRetry({ retryCount }: {
    retryCount: number;
}): number;
export declare const nanoqueryFactory: ([isAppVisible, visibilityChangeSubscribe, reconnectChangeSubscribe,]: PlatformCompat) => ({ cache, fetcher: globalFetcher, ...globalSettings }?: NanoqueryArgs) => readonly [<T = unknown, E = any>(keyInput: KeyInput, { fetcher, ...fetcherSettings }?: CommonSettings<T>) => FetcherStore<T, E>, <Data = void, Result = unknown, E_1 = any>(mutator: ManualMutator<Data, Result>, opts?: {
    throttleCalls?: boolean;
    onError?: EventTypes["onError"];
}) => MutatorStore<Data, Result, E_1>, {
    readonly __unsafeOverruleSettings: (data: CommonSettings) => void;
    readonly invalidateKeys: (keySelector: KeySelector) => void;
    readonly revalidateKeys: (keySelector: KeySelector) => void;
    readonly mutateCache: (keySelector: KeySelector, data?: unknown) => void;
}];
export {};
