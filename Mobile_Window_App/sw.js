self.addEventListener("install", event => {
  event.waitUntil(caches.open("window-app-v1").then(cache => cache.addAll(["./", "./index.html", "./manifest.json"])));
});

self.addEventListener("fetch", event => {
  const url = new URL(event.request.url);
  if (url.pathname.includes("/api/") || url.pathname.includes("/window/cmd")) return;
  event.respondWith(caches.match(event.request).then(cached => cached || fetch(event.request)));
});
