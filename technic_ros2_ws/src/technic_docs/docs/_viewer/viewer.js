// Technic docs viewer: renders the GitBook Markdown sources in the browser,
// so the drone serves the documentation without a build step or Internet.
//
// Page URL: <docs>/<lang>/?p=<page>.md#<anchor>. The page lives in the
// language directory, so relative links (../assets/x.png, other.md) resolve
// the same way as in GitBook.
(function () {
  'use strict';

  const LANG = document.documentElement.lang || 'ru';
  const OTHER_LANG = LANG === 'ru' ? 'en' : 'ru';
  const T = {
    ru: {
      title: 'Документация Technic', home: 'Веб-интерфейс', search: 'Поиск по документации',
      searching: 'Ищу…', results: 'Результаты поиска', nothing: 'Ничего не найдено',
      notFound: 'Страница не найдена', toStart: 'К началу документации',
      prev: 'Назад', next: 'Далее', menu: 'Содержание',
      ros1: 'Документация написана для версии на ROS 1. В ROS 2 команды другие: ' +
        'rosrun → ros2 run, roslaunch → ros2 launch, rostopic → ros2 topic, ' +
        'rosservice → ros2 service; топики mavros — /mavros/mavros/…',
      labels: { hint: 'Подсказка', note: 'Примечание', info: 'Информация', caution: 'Внимание',
        warning: 'Предупреждение', success: 'Готово' },
    },
    en: {
      title: 'Technic documentation', home: 'Web interface', search: 'Search the docs',
      searching: 'Searching…', results: 'Search results', nothing: 'Nothing found',
      notFound: 'Page not found', toStart: 'Back to the start',
      prev: 'Previous', next: 'Next', menu: 'Contents',
      ros1: 'This documentation is written for the ROS 1 version. ROS 2 commands differ: ' +
        'rosrun → ros2 run, roslaunch → ros2 launch, rostopic → ros2 topic, ' +
        'rosservice → ros2 service; mavros topics are /mavros/mavros/…',
      labels: { hint: 'Hint', note: 'Note', info: 'Info', caution: 'Caution',
        warning: 'Warning', success: 'Success' },
    },
  }[LANG];

  const langDir = new URL('./', location.href);          // .../docs/ru/
  const docsDir = new URL('../', langDir);               // .../docs/
  const params = new URLSearchParams(location.search);
  const page = safePage(params.get('p')) || 'README.md';

  const $ = (sel) => document.querySelector(sel);
  const content = $('#content');
  const toc = $('#toc');

  function safePage(p) {
    if (!p) return null;
    p = p.replace(/^\/+/, '');
    return /^[\w\-./]+\.md$/.test(p) && p.split('/').indexOf('..') < 0 ? p : null;
  }

  function pageUrl(lang, p, hash) {
    return new URL(lang + '/?p=' + encodeURIComponent(p).replace(/%2F/g, '/') + (hash || ''), docsDir).href;
  }

  function fetchText(url) {
    return fetch(url, { cache: 'no-cache' }).then((r) => {
      if (!r.ok) throw new Error(r.status + ' ' + url);
      return r.text();
    });
  }

  // GitBook headings may carry an explicit anchor: "## Title {#my-id}".
  const renderer = new marked.Renderer();
  renderer.heading = function (text, level, raw, slugger) {
    let id;
    const m = raw.match(/\s*\{#([^}]+)\}\s*$/);
    if (m) {
      id = m[1];
      text = text.replace(/\s*\{#[^}]+\}\s*$/, '');
    } else {
      id = slugger.slug(raw);
    }
    return '<h' + level + ' id="' + id + '">' + text +
      '<a class="anchor" href="#' + id + '" aria-hidden="true">#</a></h' + level + '>\n';
  };
  marked.setOptions({ renderer: renderer, mangle: false, headerIds: true });

  function render(md, base) {
    const div = document.createElement('div');
    div.innerHTML = marked.parse(md.replace(/<!--\s*markdownlint[^>]*-->/g, ''));
    fixLinks(div, base);
    return div;
  }

  // Rewrites links and images of a page rendered from `base` (URL of the .md file).
  function fixLinks(root, base) {
    root.querySelectorAll('[src]').forEach((el) => {
      el.src = new URL(el.getAttribute('src'), base).href;
    });
    root.querySelectorAll('a[href]').forEach((a) => {
      const href = a.getAttribute('href');
      if (!href || href.startsWith('#')) return;
      if (/^[a-z][\w+.-]*:/i.test(href)) {           // http:, tg:, mailto: ...
        a.target = '_blank';
        a.rel = 'noopener';
        return;
      }
      const url = new URL(href, base);
      const rel = url.pathname.startsWith(docsDir.pathname) ? url.pathname.slice(docsDir.pathname.length) : null;
      const m = rel && rel.match(/^(ru|en)\/(.*)$/);
      if (m) {
        let p = decodeURIComponent(m[2]);
        if (p === '' || p.endsWith('/')) p += 'README.md';
        p = p.replace(/\.html$/, '.md');
        if (p.endsWith('.md')) {
          a.href = pageUrl(m[1], p, url.hash);
          return;
        }
      }
      a.href = url.href;                             // assets: stl, pdf, images
    });
    // GitBook image-zoom plugin: click opens the full image.
    root.querySelectorAll('img.zoom').forEach((img) => {
      img.addEventListener('click', () => window.open(img.src, '_blank'));
    });
  }

  // GitBook richquotes plugin: "> **Note** text" becomes a colored box.
  function richQuotes(root) {
    root.querySelectorAll('blockquote').forEach((q) => {
      const first = q.firstElementChild;
      const strong = first && first.firstElementChild;
      if (!strong || strong.tagName !== 'STRONG' || first.firstChild !== strong) return;
      const kind = strong.textContent.trim().toLowerCase();
      if (!T.labels[kind]) return;
      q.classList.add('rq', 'rq-' + kind);
      strong.textContent = T.labels[kind];
      strong.classList.add('rq-label');
    });
  }

  let tocLinks = [];   // pages in SUMMARY order, for prev/next and search

  function loadToc() {
    const base = new URL('SUMMARY.md', langDir).href;
    return fetchText(base).then((md) => {
      const div = render(md.replace(/^#.*\n/, ''), base);
      toc.appendChild(div);
      const current = pageUrl(LANG, page);
      toc.querySelectorAll('.anchor').forEach((a) => a.remove());
      toc.querySelectorAll('a[href]').forEach((a) => {
        if (a.getAttribute('href').startsWith('#')) return;
        const u = new URL(a.href);
        const p = u.searchParams.get('p');
        if (u.pathname !== langDir.pathname || !p) return;
        tocLinks.push({ page: p, title: a.textContent.trim(), href: a.href });
        if (pageUrl(LANG, p) === current) a.classList.add('active');
      });
      const active = toc.querySelector('a.active');
      if (active) active.scrollIntoView({ block: 'center' });
    }).catch(() => {});
  }

  function loadPage() {
    const base = new URL(page, langDir).href;
    return fetchText(base).then((md) => {
      const div = render(md, base);
      richQuotes(div);
      content.replaceChildren(...div.childNodes);
      const h1 = content.querySelector('h1');
      document.title = (h1 ? h1.textContent.replace(/#$/, '') + ' — ' : '') + T.title;
      if (location.hash) {
        const target = document.getElementById(decodeURIComponent(location.hash.slice(1))) ||
          document.getElementsByName(decodeURIComponent(location.hash.slice(1)))[0];
        if (target) target.scrollIntoView();
      }
    }).catch(() => {
      content.innerHTML = '<h1>' + T.notFound + '</h1><p><code></code></p><p><a></a></p>';
      content.querySelector('code').textContent = page;
      const a = content.querySelector('p a');
      a.href = pageUrl(LANG, 'README.md');
      a.textContent = T.toStart;
      document.title = T.notFound + ' — ' + T.title;
    });
  }

  function loadFooter() {
    const base = new URL('FOOTER.md', langDir).href;
    fetchText(base).then((md) => $('#footer').replaceChildren(...render(md, base).childNodes)).catch(() => {});
  }

  function prevNext() {
    const i = tocLinks.findIndex((l) => l.page === page);
    if (i < 0) return;
    const nav = $('#pager');
    [[tocLinks[i - 1], 'prev', '← ' + T.prev], [tocLinks[i + 1], 'next', T.next + ' →']].forEach(([l, cls, label]) => {
      if (!l) return;
      const a = document.createElement('a');
      a.href = l.href;
      a.className = cls;
      a.innerHTML = '<small></small><span></span>';
      a.querySelector('small').textContent = label;
      a.querySelector('span').textContent = l.title;
      nav.appendChild(a);
    });
  }

  // Search: fetch every page of the contents once, then match plain text.
  let corpus = null;

  function plain(md) {
    return md.replace(/```[\s\S]*?```/g, ' ').replace(/<[^>]+>/g, ' ')
      .replace(/!\[[^\]]*\]\([^)]*\)/g, ' ').replace(/\[([^\]]*)\]\([^)]*\)/g, '$1')
      .replace(/[#>*_`|{}]/g, ' ').replace(/\s+/g, ' ');
  }

  function search(query) {
    query = query.trim().toLowerCase();
    if (!query) return;
    content.innerHTML = '<p class="muted">' + T.searching + '</p>';
    const pages = tocLinks.filter((l, i, all) => all.findIndex((x) => x.page === l.page) === i);
    const ready = corpus || Promise.all(pages.map((l) =>
      fetchText(new URL(l.page, langDir).href).then((md) => ({ link: l, text: plain(md) }), () => null)
    )).then((docs) => docs.filter(Boolean));
    corpus = ready;
    ready.then((docs) => {
      const words = query.split(/\s+/);
      const hits = docs.map((d) => {
        const low = d.text.toLowerCase();
        if (!words.every((w) => low.includes(w))) return null;
        const title = d.link.title.toLowerCase();
        const score = words.reduce((s, w) => s + low.split(w).length - 1 + (title.includes(w) ? 50 : 0), 0);
        const at = low.indexOf(words[0]);
        return { d: d, score: score, snippet: d.text.slice(Math.max(0, at - 80), at + 160) };
      }).filter(Boolean).sort((a, b) => b.score - a.score);

      content.innerHTML = '<h1></h1>';
      content.querySelector('h1').textContent = T.results + ': ' + query;
      if (!hits.length) {
        content.insertAdjacentHTML('beforeend', '<p class="muted">' + T.nothing + '</p>');
        return;
      }
      const list = document.createElement('ol');
      list.className = 'results';
      hits.slice(0, 50).forEach((h) => {
        const li = document.createElement('li');
        li.innerHTML = '<a></a><p></p>';
        li.querySelector('a').href = h.d.link.href;
        li.querySelector('a').textContent = h.d.link.title;
        li.querySelector('p').textContent = '… ' + h.snippet + ' …';
        list.appendChild(li);
      });
      content.appendChild(list);
      $('#pager').replaceChildren();
      document.body.classList.remove('toc-open');
    });
  }

  function setupChrome() {
    document.title = T.title;
    $('#brand').textContent = T.title;
    $('#home').textContent = T.home;
    $('#home').href = new URL('../', docsDir).href;   // /technic/
    $('#lang').textContent = OTHER_LANG.toUpperCase();
    $('#lang').href = pageUrl(OTHER_LANG, page === 'SUMMARY.md' ? 'README.md' : page);
    $('#ros1').textContent = T.ros1;
    $('#menu').setAttribute('aria-label', T.menu);
    $('#menu').addEventListener('click', () => document.body.classList.toggle('toc-open'));
    const input = $('#search');
    input.placeholder = T.search;
    $('#search-form').addEventListener('submit', (e) => {
      e.preventDefault();
      search(input.value);
    });
  }

  setupChrome();
  Promise.all([loadToc(), loadPage()]).then(prevNext);
  loadFooter();
})();
