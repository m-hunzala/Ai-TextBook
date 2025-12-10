(function (global, factory) {
  typeof exports === 'object' && typeof module !== 'undefined' ? factory(exports, require('nanostores'), require('nanoevents')) :
  typeof define === 'function' && define.amd ? define(['exports', 'nanostores', 'nanoevents'], factory) :
  (global = typeof globalThis !== 'undefined' ? globalThis : global || self, factory(global.nanoquery = {}, global.nanostores, global.nanoevents));
})(this, (function (exports, nanostores, nanoevents) { 'use strict';

  function defaultOnErrorRetry({ retryCount }) {
    return ~~((Math.random() + 0.5) * (1 << (retryCount < 8 ? retryCount : 8))) * 2e3;
  }
  const nanoqueryFactory = ([
    isAppVisible,
    visibilityChangeSubscribe,
    reconnectChangeSubscribe
  ]) => {
    const nanoquery = ({
      cache = /* @__PURE__ */ new Map(),
      fetcher: globalFetcher,
      ...globalSettings
    } = {}) => {
      const events = nanoevents.createNanoEvents();
      let focus = true;
      visibilityChangeSubscribe(() => {
        focus = isAppVisible();
        focus && events.emit(FOCUS);
      });
      reconnectChangeSubscribe(() => events.emit(RECONNECT));
      const _revalidateOnInterval = /* @__PURE__ */ new Map(), _errorInvalidateTimeouts = /* @__PURE__ */ new Map(), _runningFetches = /* @__PURE__ */ new Map();
      let rewrittenSettings = {};
      const getCachedValueByKey = (key) => {
        const fromCache = cache.get(key);
        if (!fromCache)
          return [];
        const cacheHit = (fromCache.expires || 0) > getNow();
        return cacheHit ? [fromCache.data, fromCache.error] : [];
      };
      const runFetcher = async ([key, keyParts], store, settings) => {
        if (!focus)
          return;
        const set = (v) => {
          if (store.key === key) {
            store.set(v);
            events.emit(SET_CACHE, key, v, true);
          }
        };
        const setAsLoading = (prev) => {
          const toSet = prev === void 0 ? {} : { data: prev };
          set({
            ...toSet,
            ...loading,
            promise: _runningFetches.get(key)
          });
        };
        let {
          dedupeTime = 4e3,
          cacheLifetime = Infinity,
          fetcher,
          onErrorRetry = defaultOnErrorRetry
        } = {
          ...settings,
          ...rewrittenSettings
        };
        if (cacheLifetime < dedupeTime)
          cacheLifetime = dedupeTime;
        const now = getNow();
        if (_runningFetches.has(key)) {
          if (!store.value.loading)
            setAsLoading(getCachedValueByKey(key)[0]);
          return;
        }
        let cachedValue, cachedError;
        const fromCache = cache.get(key);
        if (fromCache?.data !== void 0 || fromCache?.error) {
          [cachedValue, cachedError] = getCachedValueByKey(key);
          if ((fromCache.created || 0) + dedupeTime > now) {
            if (store.value.data != cachedValue || store.value.error != cachedError) {
              set({ ...notLoading, data: cachedValue, error: cachedError });
            }
            return;
          }
        }
        const finishTask = nanostores.startTask();
        try {
          clearTimeout(_errorInvalidateTimeouts.get(key));
          const promise = fetcher(...keyParts);
          _runningFetches.set(key, promise);
          setAsLoading(cachedValue);
          const res = await promise;
          cache.set(key, {
            data: res,
            created: getNow(),
            expires: getNow() + cacheLifetime
          });
          set({ data: res, ...notLoading });
        } catch (error) {
          settings.onError?.(error);
          const retryCount = (cache.get(key)?.retryCount || 0) + 1;
          cache.set(key, {
            error,
            created: getNow(),
            expires: getNow() + cacheLifetime,
            retryCount
          });
          if (onErrorRetry) {
            const timer = onErrorRetry({
              error,
              key,
              retryCount
            });
            if (timer)
              _errorInvalidateTimeouts.set(
                key,
                setTimeout(() => {
                  invalidateKeys(key);
                  cache.set(key, { retryCount });
                }, timer)
              );
          }
          set({ data: store.value.data, error, ...notLoading });
        } finally {
          finishTask();
          _runningFetches.delete(key);
        }
      };
      const createFetcherStore = (keyInput, {
        fetcher = globalFetcher,
        ...fetcherSettings
      } = {}) => {
        if (process.env.NODE_ENV !== "production" && !fetcher) {
          throw new Error(
            "You need to set up either global fetcher of fetcher in createFetcherStore"
          );
        }
        const fetcherStore = nanostores.map({
          ...notLoading
        }), settings = { ...globalSettings, ...fetcherSettings, fetcher };
        fetcherStore._ = fetcherSymbol;
        fetcherStore.invalidate = () => {
          const { key } = fetcherStore;
          if (key) {
            invalidateKeys(key);
          }
        };
        fetcherStore.revalidate = () => {
          const { key } = fetcherStore;
          if (key) {
            revalidateKeys(key);
          }
        };
        fetcherStore.mutate = (data) => {
          const { key } = fetcherStore;
          if (key) {
            mutateCache(key, data);
          }
        };
        fetcherStore.fetch = async () => {
          let resolve;
          const promise = new Promise((r) => resolve = r);
          const unsub = fetcherStore.listen(({ error, data }) => {
            if (error !== void 0)
              resolve({ error });
            if (data !== void 0)
              resolve({ data });
          });
          return promise.finally(unsub);
        };
        let keysInternalUnsub, prevKey, prevKeyParts, keyUnsub, keyStore;
        let evtUnsubs = [];
        nanostores.onStart(fetcherStore, () => {
          const firstRun = !keysInternalUnsub;
          [keyStore, keysInternalUnsub] = getKeyStore(keyInput);
          keyUnsub = keyStore.subscribe((currentKeys) => {
            if (currentKeys) {
              const [newKey, keyParts] = currentKeys;
              fetcherStore.key = newKey;
              runFetcher([newKey, keyParts], fetcherStore, settings);
              prevKey = newKey;
              prevKeyParts = keyParts;
            } else {
              fetcherStore.key = prevKey = prevKeyParts = void 0;
              fetcherStore.set({ ...notLoading });
            }
          });
          const currentKeyValue = keyStore.get();
          if (currentKeyValue) {
            [prevKey, prevKeyParts] = currentKeyValue;
            if (firstRun)
              handleNewListener();
          }
          const {
            revalidateInterval = 0,
            revalidateOnFocus,
            revalidateOnReconnect
          } = settings;
          const runRefetcher = () => {
            if (prevKey)
              runFetcher([prevKey, prevKeyParts], fetcherStore, settings);
          };
          if (revalidateInterval > 0) {
            _revalidateOnInterval.set(
              keyInput,
              setInterval(runRefetcher, revalidateInterval)
            );
          }
          if (revalidateOnFocus)
            evtUnsubs.push(events.on(FOCUS, runRefetcher));
          if (revalidateOnReconnect)
            evtUnsubs.push(events.on(RECONNECT, runRefetcher));
          const cacheKeyChangeHandler = (keySelector) => {
            if (prevKey && testKeyAgainstSelector(prevKey, keySelector)) {
              runFetcher([prevKey, prevKeyParts], fetcherStore, settings);
            }
          };
          evtUnsubs.push(
            events.on(INVALIDATE_KEYS, cacheKeyChangeHandler),
            events.on(REVALIDATE_KEYS, cacheKeyChangeHandler),
            events.on(SET_CACHE, (keySelector, data, full) => {
              if (prevKey && testKeyAgainstSelector(prevKey, keySelector) && fetcherStore.value !== data && fetcherStore.value.data !== data) {
                fetcherStore.set(
                  full ? data : { data, ...notLoading }
                );
              }
            })
          );
        });
        const handleNewListener = () => {
          if (prevKey && prevKeyParts)
            runFetcher([prevKey, prevKeyParts], fetcherStore, settings);
        };
        const originListen = fetcherStore.listen;
        fetcherStore.listen = (listener) => {
          const unsub = originListen(listener);
          listener(fetcherStore.value);
          handleNewListener();
          return unsub;
        };
        nanostores.onStop(fetcherStore, () => {
          fetcherStore.value = { ...notLoading };
          keysInternalUnsub?.();
          evtUnsubs.forEach((fn) => fn());
          evtUnsubs = [];
          keyUnsub?.();
          clearInterval(_revalidateOnInterval.get(keyInput));
        });
        return fetcherStore;
      };
      const iterOverCache = (keySelector, cb) => {
        for (const key of cache.keys()) {
          if (testKeyAgainstSelector(key, keySelector))
            cb(key);
        }
      };
      const invalidateKeys = (keySelector) => {
        iterOverCache(keySelector, (key) => {
          cache.delete(key);
        });
        events.emit(INVALIDATE_KEYS, keySelector);
      };
      const revalidateKeys = (keySelector) => {
        iterOverCache(keySelector, (key) => {
          const cached = cache.get(key);
          if (cached) {
            cache.set(key, { ...cached, created: -Infinity });
          }
        });
        events.emit(REVALIDATE_KEYS, keySelector);
      };
      const mutateCache = (keySelector, data) => {
        iterOverCache(keySelector, (key) => {
          if (data === void 0)
            cache.delete(key);
          else {
            cache.set(key, {
              data,
              created: getNow(),
              expires: getNow() + (globalSettings.cacheLifetime ?? 8e3)
            });
          }
        });
        events.emit(SET_CACHE, keySelector, data);
      };
      function createMutatorStore(mutator, opts) {
        const { throttleCalls, onError } = opts ?? {
          throttleCalls: true,
          onError: globalSettings?.onError
        };
        const mutate = async (data) => {
          if (throttleCalls && store.value?.loading)
            return;
          const newMutator = rewrittenSettings.fetcher ?? mutator;
          const keysToInvalidate = [], keysToRevalidate = [];
          const safeKeySet = (k, v) => {
            if (store.lc) {
              store.setKey(k, v);
            }
          };
          try {
            store.set({
              error: void 0,
              data: void 0,
              mutate,
              ...loading
            });
            const result = await newMutator({
              data,
              invalidate: (key) => {
                keysToInvalidate.push(key);
              },
              revalidate: (key) => {
                keysToRevalidate.push(key);
              },
              getCacheUpdater: (key, shouldRevalidate = true) => [
                (newVal) => {
                  mutateCache(key, newVal);
                  if (shouldRevalidate) {
                    keysToRevalidate.push(key);
                  }
                },
                cache.get(key)?.data
              ]
            });
            safeKeySet("data", result);
            return result;
          } catch (error) {
            onError?.(error);
            safeKeySet("error", error);
            store.setKey("error", error);
          } finally {
            safeKeySet("loading", false);
            keysToInvalidate.forEach(invalidateKeys);
            keysToRevalidate.forEach(revalidateKeys);
          }
        };
        const store = nanostores.map({
          mutate,
          ...notLoading
        });
        nanostores.onStop(
          store,
          () => store.set({ mutate, ...notLoading })
        );
        store.mutate = mutate;
        return store;
      }
      const __unsafeOverruleSettings = (data) => {
        if (process.env.NODE_ENV !== "test") {
          console.warn(
            `You should only use __unsafeOverruleSettings in test environment`
          );
        }
        rewrittenSettings = data;
      };
      return [
        createFetcherStore,
        createMutatorStore,
        { __unsafeOverruleSettings, invalidateKeys, revalidateKeys, mutateCache }
      ];
    };
    function isSomeKey(key) {
      return typeof key === "string" || typeof key === "number" || key === true;
    }
    const getKeyStore = (keys) => {
      if (isSomeKey(keys))
        return [
          nanostores.atom(["" + keys, [keys]]),
          () => {
          }
        ];
      const keyParts = [];
      const $key = nanostores.atom(null);
      const keysAsStoresToIndexes = /* @__PURE__ */ new Map();
      const setKeyStoreValue = () => {
        if (keyParts.some((v) => v === null || v === void 0 || v === false)) {
          $key.set(null);
        } else {
          $key.set([keyParts.join(""), keyParts]);
        }
      };
      for (let i = 0; i < keys.length; i++) {
        const keyOrStore = keys[i];
        if (isSomeKey(keyOrStore)) {
          keyParts.push(keyOrStore);
        } else {
          keyParts.push(null);
          keysAsStoresToIndexes.set(keyOrStore, i);
        }
      }
      const storesAsArray = [...keysAsStoresToIndexes.keys()];
      const $storeKeys = nanostores.batched(storesAsArray, (...storeValues) => {
        for (let i = 0; i < storeValues.length; i++) {
          const store = storesAsArray[i], partIndex = keysAsStoresToIndexes.get(store);
          keyParts[partIndex] = store._ === fetcherSymbol ? store.value && "data" in store.value ? store.key : null : storeValues[i];
        }
        setKeyStoreValue();
      });
      setKeyStoreValue();
      return [$key, $storeKeys.subscribe(noop)];
    };
    function noop() {
    }
    const FOCUS = 1, RECONNECT = 2, INVALIDATE_KEYS = 3, REVALIDATE_KEYS = 4, SET_CACHE = 5;
    const testKeyAgainstSelector = (key, selector) => {
      if (Array.isArray(selector))
        return selector.includes(key);
      else if (typeof selector === "function")
        return selector(key);
      else
        return key === selector;
    };
    const getNow = () => (/* @__PURE__ */ new Date()).getTime();
    const fetcherSymbol = Symbol();
    const loading = { loading: true }, notLoading = { loading: false };
    return nanoquery;
  };

  const subscribe = (name, fn) => {
    const isServer = typeof window === "undefined";
    if (!isServer) {
      addEventListener(name, fn);
    }
  };
  const browserCompat = [
    () => !document.hidden,
    (cb) => subscribe("visibilitychange", cb),
    (cb) => subscribe("online", cb)
  ];

  const nanoquery = nanoqueryFactory(browserCompat);

  exports.nanoquery = nanoquery;
  exports.onErrorRetry = defaultOnErrorRetry;

  Object.defineProperty(exports, Symbol.toStringTag, { value: 'Module' });

}));
