export type { KeyInput, KeySelector, Fetcher, OnErrorRetry, CommonSettings, NanoqueryArgs, FetcherValue, FetcherStore, FetcherStoreCreator, ManualMutator, MutateCb, MutatorStore, } from './factory';
export { defaultOnErrorRetry as onErrorRetry } from './factory';
export declare const nanoquery: ({ cache, fetcher: globalFetcher, ...globalSettings }?: import('./factory').NanoqueryArgs) => readonly [<T = unknown, E = any>(keyInput: import('./factory').KeyInput, { fetcher, ...fetcherSettings }?: import('./factory').CommonSettings<T>) => import('./factory').FetcherStore<T, E>, <Data = void, Result = unknown, E_1 = any>(mutator: import('./factory').ManualMutator<Data, Result>, opts?: {
    throttleCalls?: boolean | undefined;
    onError?: ((error: unknown) => void) | undefined;
} | undefined) => import('./factory').MutatorStore<Data, Result, E_1>, {
    readonly __unsafeOverruleSettings: (data: import('./factory').CommonSettings<unknown>) => void;
    readonly invalidateKeys: (keySelector: import('./factory').KeySelector) => void;
    readonly revalidateKeys: (keySelector: import('./factory').KeySelector) => void;
    readonly mutateCache: (keySelector: import('./factory').KeySelector, data?: unknown) => void;
}];
