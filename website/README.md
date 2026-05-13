# gmt_dos-actors website

Static showcase site for the [GMT Integrated Modeling Computing Framework](https://github.com/rconan/dos-actors).

## Live URL

**http://dos-actors.gmto.im**

Hosted on AWS S3 (`dos-actors.gmto.im`) with a Route 53 alias A record pointing to the S3 static website endpoint (`s3-website-us-east-1.amazonaws.com`).

## Structure

```
website/
└── index.html   # single-file static site (HTML + CSS + JS)
```

All styles, scripts, and content are self-contained in `index.html`. External dependencies loaded from CDN:

- [highlight.js](https://highlightjs.org/) — Rust syntax highlighting
- [Google Fonts](https://fonts.google.com/) — Inter + JetBrains Mono

## Deploy

```bash
aws s3 cp website/index.html s3://dos-actors.gmto.im/index.html --content-type "text/html"
```

Requires AWS CLI configured with write access to the `dos-actors.gmto.im` bucket.

## Infrastructure

| Resource | Details |
|----------|---------|
| S3 bucket | `dos-actors.gmto.im` (us-east-1) |
| Static website hosting | index document: `index.html` |
| Public access | bucket policy: `s3:GetObject` for `*` |
| DNS | Route 53 hosted zone `gmto.im` — alias A record → S3 website endpoint |

To add HTTPS, front the bucket with a CloudFront distribution using an ACM certificate for `dos-actors.gmto.im` and update the Route 53 record to point to CloudFront.
